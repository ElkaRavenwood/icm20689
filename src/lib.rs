/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]

use embedded_hal as hal;
use hal::blocking::delay::DelayMs;
use hal::digital::v2::OutputPin;
use core::convert::TryInto;

#[cfg(feature = "rttdebug")]
use panic_rtt_core::rprintln;

mod interface;
pub use interface::{I2cInterface, SensorInterface, SpiInterface};

/// Errors in this crate
#[derive(Debug)]
pub enum Error<CommE, PinE> {
    /// Sensor communication error
    Comm(CommE),
    /// Pin setting error
    Pin(PinE),
    /// Unrecognized chip ID
    UnknownChipId,
    /// Sensor not responding
    Unresponsive,
}

pub struct Builder {}

impl Builder {
    /// Create a new driver using I2C interface
    pub fn new_i2c<I2C, CommE>(&self, i2c: I2C, address: u8) -> ICM20689<I2cInterface<I2C>>
    where
        I2C: hal::blocking::i2c::Write<Error = CommE>
            + hal::blocking::i2c::Read<Error = CommE>
            + hal::blocking::i2c::WriteRead<Error = CommE>,
        CommE: core::fmt::Debug,
    {
        let iface = interface::I2cInterface::new(i2c, address);
        ICM20689::new_with_interface(iface)
    }

    /// Create a new driver using SPI interface
    pub fn new_spi<SPI, CSN, CommE, PinE>(spi: SPI, csn: CSN) -> ICM20689<SpiInterface<SPI, CSN>>
    where
        SPI: hal::blocking::spi::Transfer<u8, Error = CommE>
            + hal::blocking::spi::Write<u8, Error = CommE>,
        CSN: OutputPin<Error = PinE>,
        CommE: core::fmt::Debug,
        PinE: core::fmt::Debug,
    {
        let iface = interface::SpiInterface::new(spi, csn);
        ICM20689::new_with_interface(iface)
    }
}

pub struct ICM20689<SI> {
    pub(crate) si: SI,
    pub(crate) gyro_scale: f32,
    pub(crate) accel_scale: f32,
}

impl<SI, CommE, PinE> ICM20689<SI>
where
    SI: SensorInterface<InterfaceError = Error<CommE, PinE>>,
{
    pub(crate) fn new_with_interface(sensor_interface: SI) -> Self {
        Self {
            si: sensor_interface,
            gyro_scale: 0.0,
            accel_scale: 0.0,
        }
    }

    /// Read the sensor identifier and return true if they match a supported value
    /// where supported values are for the ICM20602, ICM20608 and ICM20689 chips
    pub fn check_identity(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<bool, SI::InterfaceError> {
        for _ in 0..5 {
            let chip_id = self.si.register_read(REG_WHO_AM_I)?;
            match chip_id {
                ICM20602_WAI | ICM20608_WAI | ICM20689_WAI => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("found device: 0x{:0x}  ", chip_id);
                    return Ok(true);
                }
                _ => {
                    #[cfg(feature = "rttdebug")]
                    rprintln!("bogus whoami: 0x{:0x}  ", chip_id);
                }
            }

            delay_source.delay_ms(10);
        }

        Ok(false)
    }

    /// Perform a soft reset on the sensor
    /// This is required for proper sensor initialization and clock selection
    pub fn soft_reset(
        &mut self,
        delay_source: &mut impl DelayMs<u8>,
    ) -> Result<(), SI::InterfaceError> {
        /// disable I2C interface if we're using SPI
        const I2C_IF_DIS: u8 = 1 << 4;

        /// reset the device
        const PWR_DEVICE_RESET: u8 = 1 << 7; // 0x80 : 0b10000000;

        /// Auto-select between internal relaxation oscillator and
        /// gyroscope MEMS oscillator to use the best available source
        const CLKSEL_AUTO: u8 = 0x01;
        const SENSOR_ENABLE_ALL: u8 = 0x00;

        // self.dev.write(Register::PWR_MGMT_1, 0x80)?;
        // // get stable time source;
        // // Auto select clock source to be PLL gyroscope reference if ready
        // // else use the internal oscillator, bits 2:0 = 001
        // self.dev.write(Register::PWR_MGMT_1, 0x01)?;
        // // Enable all sensors
        // self.dev.write(Register::PWR_MGMT_2, 0x00)?;
        // delay.delay_ms(200);

        self.si.register_write(REG_PWR_MGMT_1, PWR_DEVICE_RESET)?;
        //reset can take up to 100 ms?
        delay_source.delay_ms(110);

        let mut reset_success = false;
        for _ in 0..10 {
            //The reset bit automatically clears to 0 once the reset is done.
            if let Ok(reg_val) = self.si.register_read(REG_PWR_MGMT_1) {
                if reg_val & PWR_DEVICE_RESET == 0 {
                    reset_success = true;
                    break;
                }
            }
            delay_source.delay_ms(10);
        }
        if !reset_success {
            #[cfg(feature = "rttdebug")]
            rprintln!("couldn't read REG_PWR_MGMT_1");
            return Err(Error::Unresponsive);
        }

        if self.si.using_spi() {
            // disable i2c just after reset
            self.si.register_write(REG_USER_CTRL, I2C_IF_DIS)?;
        }

        //setup the automatic clock selection
        self.si.register_write(REG_PWR_MGMT_1, CLKSEL_AUTO)?;
        //enable accel and gyro
        self.si.register_write(REG_PWR_MGMT_2, SENSOR_ENABLE_ALL)?;

        delay_source.delay_ms(200);

        Ok(())
    }

    /// give the sensor interface a chance to set up
    pub fn setup(&mut self, delay_source: &mut impl DelayMs<u8>) -> Result<(), SI::InterfaceError> {
        // const DLPF_CFG_1: u8 = 0x01;
        //const SIG_COND_RST: u8 = 1 << 0;
        const FIFO_RST: u8 = 1 << 2;
        const DMP_RST: u8 = 1 << 3;

        // note that id check before reset will fail
        self.soft_reset(delay_source)?;
        let supported = self.check_identity(delay_source)?;
        if !supported {
            return Err(Error::UnknownChipId);
        }

        //TODO Configure the Digital Low Pass Filter (DLPF)
        // self.si.register_write(Self::REG_CONFIG, A_DLPF_CFG)?;
        // //set the sample frequency
        // self.si.register_write(Self::REG_SMPLRT_DIV, 0x01)?;

        // disable interrupt pin
        self.si.register_write(REG_INT_ENABLE, 0x00)?;

        // disable FIFO
        //self.si.register_write(REG_FIFO_EN, 0x00)?;

        //enable FIFO for gyro and accel only:
        self.si.register_write(REG_FIFO_EN, 0x7C)?;

        //TODO what about SIG_COND_RST  ?
        let ctrl_flags = FIFO_RST | DMP_RST;
        self.si.register_write(REG_USER_CTRL, ctrl_flags)?;

        //configure some default ranges
        self.set_accel_range(AccelRange::default())?;
        self.set_gyro_range(GyroRange::default())?;

        // VARIATION 1
        // check if accelerometer works
        if !self.self_test_accel()? {
            panic!();
        };
        // check if gyroscope works
        if !self.self_test_gyro()? {
            panic!();
        };

        // VARIATION 2
        //check if accelerometer works and calibrate
        self.self_test_calibration(REG_ACCEL_CONFIG,REG_SELF_TEST_X_ACCEL, REG_XA_OFFSET_L, 0x80)?; // X-Axis
        self.self_test_calibration(REG_ACCEL_CONFIG,REG_SELF_TEST_Y_ACCEL, REG_YA_OFFSET_L, 0x40)?; // Y-Axis
        self.self_test_calibration(REG_ACCEL_CONFIG,REG_SELF_TEST_Z_ACCEL, REG_ZA_OFFSET_L, 0x20)?; // Z-Axis

        //check if gyroscope works and calibrate
        self.self_test_calibration(REG_GYRO_CONFIG,REG_SELF_TEST_X_GYRO, REG_XG_OFFS_USRL, 0x80)?; // X-Axis
        self.self_test_calibration(REG_GYRO_CONFIG,REG_SELF_TEST_Z_GYRO, REG_YG_OFFS_USRL, 0x40)?; // Z-Axis
        self.self_test_calibration(REG_GYRO_CONFIG,REG_SELF_TEST_Y_GYRO, REG_ZG_OFFS_USRL, 0x20)?; // Y-Axis

        // VARIATION 3
        //check if accelerometer works and calibrate
        self.self_test_calibration_2(REG_ACCEL_CONFIG,REG_SELF_TEST_X_ACCEL, REG_XA_OFFSET_L, 0x80)?; // X-Axis
        self.self_test_calibration_2(REG_ACCEL_CONFIG,REG_SELF_TEST_Y_ACCEL, REG_YA_OFFSET_L, 0x40)?; // Y-Axis
        self.self_test_calibration_2(REG_ACCEL_CONFIG,REG_SELF_TEST_Z_ACCEL, REG_ZA_OFFSET_L, 0x20)?; // Z-Axis

        //check if gyroscope works and calibrate
        self.self_test_calibration_2(REG_GYRO_CONFIG,REG_SELF_TEST_X_GYRO, REG_XG_OFFS_USRL, 0x80)?; // X-Axis
        self.self_test_calibration_2(REG_GYRO_CONFIG,REG_SELF_TEST_Z_GYRO, REG_YG_OFFS_USRL, 0x40)?; // Z-Axis
        self.self_test_calibration_2(REG_GYRO_CONFIG,REG_SELF_TEST_Y_GYRO, REG_ZG_OFFS_USRL, 0x20)?; // Y-Axis


        // VARIATION 4: CALIBRATION
        // NOTE NOT HANDLING ERROR
        // Does it by 100 elements
        // Calibrate Gyro
        // This would be for holding it steady
        self.calibrate_by_n_elements(REG_GYRO_XOUT_H, REG_XG_OFFS_USRH, 100)?;
        self.calibrate_by_n_elements(REG_GYRO_XOUT_L, REG_XG_OFFS_USRL, 100)?;
        self.calibrate_by_n_elements(REG_GYRO_YOUT_H, REG_YG_OFFS_USRH, 100)?;
        self.calibrate_by_n_elements(REG_GYRO_YOUT_L, REG_YG_OFFS_USRL, 100)?;
        self.calibrate_by_n_elements(REG_GYRO_ZOUT_H, REG_ZG_OFFS_USRH, 100)?;
        self.calibrate_by_n_elements(REG_GYRO_ZOUT_L, REG_ZG_OFFS_USRL, 100)?;

        // Calibrate Accel
        // Ideally we want to do this while holding it against, toward and perpendicular to gravity for each of these
        // But we can't flip a robot upside down
        self.calibrate_by_n_elements(REG_ACCEL_XOUT_H, REG_XA_OFFSET_H, 100)?;
        self.calibrate_by_n_elements(REG_ACCEL_XOUT_L, REG_XA_OFFSET_L, 100)?;
        self.calibrate_by_n_elements(REG_ACCEL_YOUT_H, REG_YA_OFFSET_H, 100)?;
        self.calibrate_by_n_elements(REG_ACCEL_YOUT_L, REG_YA_OFFSET_L, 100)?;
        self.calibrate_by_n_elements(REG_ACCEL_ZOUT_H, REG_ZA_OFFSET_H, 100)?;
        self.calibrate_by_n_elements(REG_ACCEL_ZOUT_L, REG_ZA_OFFSET_L, 100)?;

        Ok(())
    }

    /// Set the full scale range of the accelerometer
    pub fn set_accel_range(&mut self, range: AccelRange) -> Result<(), SI::InterfaceError> {
        self.accel_scale = range.scale();
        self.si.register_write(REG_ACCEL_CONFIG, (range as u8) << 3)
    }

    /// Set the full scale range of the gyroscope
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), SI::InterfaceError> {
        self.gyro_scale = range.scale();
        self.si.register_write(REG_GYRO_CONFIG, (range as u8) << 2)
    }

    /// Get raw accelerometer data
    pub fn get_raw_accel(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        self.si.read_vec3_i16(REG_ACCEL_START)
    }

    /// Get raw gyroscope data
    pub fn get_raw_gyro(&mut self) -> Result<[i16; 3], SI::InterfaceError> {
        self.si.read_vec3_i16(REG_GYRO_START)
    }

    /// Get scaled accelerometer data
    pub fn get_scaled_accel(&mut self) -> Result<[f32; 3], SI::InterfaceError> {
        let raw_accel = self.get_raw_accel()?;
        Ok([
            self.accel_scale * (raw_accel[0] as f32),
            self.accel_scale * (raw_accel[1] as f32),
            self.accel_scale * (raw_accel[2] as f32),
        ])
    }

    /// Get scaled gyroscope data
    pub fn get_scaled_gyro(&mut self) -> Result<[f32; 3], SI::InterfaceError> {
        let raw_gyro = self.get_raw_gyro()?;
        Ok([
            self.gyro_scale * (raw_gyro[0] as f32),
            self.gyro_scale * (raw_gyro[1] as f32),
            self.gyro_scale * (raw_gyro[2] as f32),
        ])
    }

    /// Self test interpretation 1:
    /// Recall that self test is just to see if the sensors work
    /// Assumes that the result of the self test is whether or not this passes/fails
    /// Assumes the pass value is 1.
    fn self_test_gyro(&mut self) -> Result<bool, SI::InterfaceError> {
        // read from self test
        // enables test for all axes
        
        self.si.register_write(REG_GYRO_CONFIG, 0xE0)?;
        // TODO wait for some time
        // thread::sleep(core::time::Duration::from_secs(5));
        // disables test for specific axis
        self.si.register_write(REG_GYRO_CONFIG, 0x0)?;
        // check if this value is a pass
        let val = self.si.register_read(REG_SELF_TEST_X_GYRO)? + self.si.register_read(REG_SELF_TEST_Y_GYRO)?+ self.si.register_read(REG_SELF_TEST_Z_GYRO)?;

        // read and return value from self test register. If all pass (ie the accelerometer works), then this function returns true
        // return
        Ok(val == 3)
    }

    /// Self test interpretation 1:
    /// Recall that self test is just to see if the sensors work
    /// Assumes that the result of the self test is whether or not this passes/fails
    /// Assumes the pass value is 1.
    fn self_test_accel(&mut self) -> Result<bool, SI::InterfaceError> {
        // read from self test
        // enables test for all axes
        
        self.si.register_write(REG_ACCEL_CONFIG, 0xE0)?;
        // TODO wait for some time
        // thread::sleep(core::time::Duration::from_secs(5));
        // disables test for specific axis
        self.si.register_write(REG_ACCEL_CONFIG, 0x0)?;
        // check if this value is a pass
        let val = self.si.register_read(REG_SELF_TEST_X_ACCEL)? == 0x1 && 
            self.si.register_read(REG_SELF_TEST_X_ACCEL)? == self.si.register_read(REG_SELF_TEST_Y_ACCEL)? && 
            self.si.register_read(REG_SELF_TEST_X_ACCEL)? == self.si.register_read(REG_SELF_TEST_Z_ACCEL)?;

        // read and return value from self test register. If all pass (ie the accelerometer works), then this function returns true
        // return
        Ok(val)
    }

    /// calibration based on self test
    /// This assumes that the self test response is self-test response = Sensor output with self-test enabled – Sensor output with self-test disabled
    /// Documentation says that it passes' if the value is less than the product specification state
    /// The product specifications don't actually say anything (or give concrete values)
    /// So we can probably just use the self-test response to adjust the offsets of the gyroscopes/accelerometers
    /// Assumes that the self-test is overwritten from factory settings AFTER self test has been enabled, then disabled
    /// 
    /// calibrates accelerometer or gyroscope given the configuration, self test, offset registers and the bit in the config register to enable
    fn self_test_calibration(&mut self, config: u8, self_test_register: u8, offset_register: u8, enable_bit: u8) -> Result<(), SI::InterfaceError> {
        
        // enables test for specific axis
        self.si.register_write(config, enable_bit)?;
        // TODO wait for a bit
        // disables test for specific axis
        self.si.register_write(config, enable_bit)?;
        let self_test = self.si.register_read(self_test_register)?;
        // write to offset differences in self test
        self.si.register_write(offset_register, self_test)?;
        // return
        Ok(())
    }

    /// calibration based on self test
    /// This assumes that the self test response is self-test response = Sensor output with self-test enabled – Sensor output with self-test disabled
    /// Documentation says that it passes' if the value is less than the product specification state
    /// The product specifications don't actually say anything (or give concrete values)
    /// So we can probably just use the DIFFERENCE IN SELF-TEST-RESPONS FROM THE FACTORY_SELF_TEST  to adjust the offsets of the gyroscopes/accelerometers
    /// Assumes that the self-test is overwritten from factory settings AFTER self test has been enabled, then disabled
    /// 
    /// calibrates accelerometer or gyroscope given the configuration, self test, offset registers and the bit in the config register to enable
    fn self_test_calibration_2(&mut self, config: u8, self_test_register: u8, offset_register: u8, enable_bit: u8) -> Result<(), SI::InterfaceError> {
        
        // read from self test
        let factory_self_test = self.si.register_read(self_test_register)?;
        // enables test for specific axis
        self.si.register_write(config, enable_bit)?;
        // TODO wait for a bit
        // disables test for specific axis
        self.si.register_write(config, enable_bit)?;
        // get difference of self tests
        let self_test = factory_self_test - self.si.register_read(self_test_register)?;
        // write to offset differences in self test
        self.si.register_write(offset_register, self_test)?;
        // return
        Ok(())
    }

    /// calibrate given read register, offset registers and the number of samples to take
    fn calibrate_by_n_elements (&mut self, read_reg: u8, offset_reg: u8, n: i32) -> Result<(), SI::InterfaceError>  {
        let mut avg:i32 = 0;
        let mut i = 0;
        while i < n {
            // TODO figure how to check if reads were successful
            avg = avg + i32:: from(self.si.register_read(read_reg)?);
            i = i + 1;
        }
        // get average
        avg = avg / n;
        // adjust offset
        self.si.register_write(offset_reg, avg.try_into().unwrap())?;
        // return
        Ok(())
    }

}

/// Common registers
/// These are all possible registers

/// The following are for: self testing the gyroscope
/// The value in bits 7:0 for this register indicates the self-test output
/// generated during manufacturing tests. This value is to be used to check
/// against subsequent self-test outputs performed by the end user.
const REG_SELF_TEST_X_GYRO: u8 = 0x00;
const REG_SELF_TEST_Y_GYRO: u8 = 0x01;
const REG_SELF_TEST_Z_GYRO: u8 = 0x02;

/// The following are for: self testing the accelerometer
/// The value in bits 7:0 for this register indicates the self-test output
/// generated during manufacturing tests. This value is to be used to check
/// against subsequent self-test outputs performed by the end user.
const REG_SELF_TEST_X_ACCEL: u8 = 0x0D;
const REG_SELF_TEST_Y_ACCEL: u8 = 0x0E;
const REG_SELF_TEST_Z_ACCEL: u8 = 0x0F;

/// The following are for: adjusting the gyroscope's offset
/// Bits 15:8(H) and 7:0(L) of the 16-bit offset of X gyroscope (2’s complement). This
/// register is used to remove DC bias from the sensor output. The value in
/// this register is added to the gyroscope sensor value before going into
/// the sensor register.
const REG_XG_OFFS_USRH: u8 = 0x13;
const REG_XG_OFFS_USRL: u8 = 0x14;
const REG_YG_OFFS_USRH: u8 = 0x15;
const REG_YG_OFFS_USRL: u8 = 0x16;
const REG_ZG_OFFS_USRH: u8 = 0x17;
const REG_ZG_OFFS_USRL: u8 = 0x18;

/// The following is for: SAMPLE RATE DIVIDER
/// Divides the internal sample rate (see register CONFIG) to generate the sample
/// rate that controls sensor data output rate, FIFO sample rate. NOTE: This register
/// is only effective when FCHOICE_B register bits are 2’b00, and (0 < DLPF_CFG < 7).
/// This is the update rate of the sensor register:
/// SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
/// Where INTERNAL_SAMPLE_RATE = 1kHz
// const REG_SMPLRT_DIV: u8 = 0x19;

/// The following are for:
// const REG_CONFIG: u8 = 0x1A; // has low pass filter

/// The following is for: GYROSCOPE CONFIGURATION
/// Bits 7:5 are for self-test XYZ respectively
/// Bits 4:3 are for the gyro full scale select
/// Bits 1:0 are for the DLPF bypass    
const REG_GYRO_CONFIG: u8 = 0x1B;

///
/// The following is for: ACCELEROMETER CONFIGURATION
/// Bits 7:5 are for self-test XYZ respectively
/// Bits 4:3 are for the accelerometer full scale select
const REG_ACCEL_CONFIG: u8 = 0x1C;

///
/// The following is also for: ACCELEROMETER CONFIGURATION
/// Bits 5:4 are for averaging filter settings in low power accelerometer mode
/// Bits 3 are for the DLPF bypass
/// Bits 2:0 are for accelerometer low pass filtering
// const REG_ACCEL_CONFIG_2: u8 = 0x1D;

/// The following is for: LOW POWER MODE CONFIGURATION
/// Bit 7 When set to ‘1’ low-power gyroscope mode is enabled. Default
/// setting is ‘0’
/// Bits 6:4 Averaging filter configuration for low-power gyroscope mode.
/// Default setting is ‘000’
// const REG_LP_MODE_CFG: u8 = 0x1E;

///
/// The following is for: WAKE ON MOTION THRESHOLD
/// Bits 7:0 This register holds the threshold value for the Wake on Motion Interrupt for
/// accelerometer.
// const REG_ACCEL_WOM_THR: u8 = 0x1F;

///
/// The following are for: FIFO ENABLE
const REG_FIFO_EN: u8 = 0x23;

///
/// The following is for: FSYNC interrupt status
// const REG_FSYNC_INT: u8 = 0x36;

///
/// The following is for: INT/DRDY PIN / BYPASS ENABLE CONFIGURATION
// const REG_INT_PIN_CFG: u8 = 0x37;

///
/// The following is for: Interrupt enable
const REG_INT_ENABLE: u8 = 0x38;

///
/// The following is for: DMP Interrupt status
// const REG_DMP_INT_STATUS: u8 = 0x39;

///
/// The following is for: Interrupt status
// const REG_INT_STATUS: u8 = 0x3A;

///
/// The following are for: Accelerometer measurements
/// These are read only registers
const REG_ACCEL_XOUT_H: u8 = 0x3B; // contains the higher BITS
const REG_ACCEL_XOUT_L: u8 = 0x3C;
const REG_ACCEL_YOUT_H: u8 = 0x3D;
const REG_ACCEL_YOUT_L: u8 = 0x3E;
const REG_ACCEL_ZOUT_H: u8 = 0x3F;
const REG_ACCEL_ZOUT_L: u8 = 0x40;

///
/// The following are for: temperature measurements
/// These are read only registers
// const REG_TEMP_OUT_H: u8 = 0x41;
// const REG_TEMP_OUT_L: u8 = 0x42;

///
/// The following are for: gyroscope measurements
/// These are read only registers
const REG_GYRO_XOUT_H: u8 = 0x43;
const REG_GYRO_XOUT_L: u8 = 0x44;
const REG_GYRO_YOUT_H: u8 = 0x45;
const REG_GYRO_YOUT_L: u8 = 0x46;
const REG_GYRO_ZOUT_H: u8 = 0x47;
const REG_GYRO_ZOUT_L: u8 = 0x48;

///
/// The following are for: SIGNAL PATH RESET
// const REG_SIGNAL_PATH_RESET: u8 = 0x68;

///
/// The following are for: ACCELEROMETER INTELLIGENCE CONTROL
/// like the wake on motion
/// Bit 7 enables or disables this feature
// const REG_ACCEL_INTEL_CTRL: u8 = 0x69;

///
/// The following is for: user control
const REG_USER_CTRL: u8 = 0x6A;

///
/// The following are for: power management
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;

/// The following are for: FIFO count registers
// const REG_FIFO_COUNTH: u8 = 0x72;
// const REG_FIFO_COUNTL: u8 = 0x73;

///
/// The following is for: FIFO read/write
/// (from FIFO buffer)
// const REG_FIFO_R_W: u8 = 0x74;

///
/// The following is for: verifying identity of device
const REG_WHO_AM_I: u8 = 0x75;

///
/// The following are for: accelerometer offset
const REG_XA_OFFSET_H: u8 = 0x77;
const REG_XA_OFFSET_L: u8 = 0x78;
const REG_YA_OFFSET_H: u8 = 0x7A;
const REG_YA_OFFSET_L: u8 = 0x7B;
const REG_ZA_OFFSET_H: u8 = 0x7D;
const REG_ZA_OFFSET_L: u8 = 0x7E;

/// The following are for:
/// whatever the original author intended
const REG_ACCEL_START: u8 = REG_ACCEL_XOUT_H;
const REG_GYRO_START: u8 = REG_GYRO_XOUT_H;

/// Device IDs for various supported devices
const ICM20602_WAI: u8 = 0x12;
const ICM20608_WAI: u8 = 0xAF;
const ICM20689_WAI: u8 = 0x98; // this is ours

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// The gyroscope has a programmable full-scale range of ±250, ±500, ±1000, or ±2000 degrees/sec.
pub enum GyroRange {
    /// ±250
    Range_250dps = 0b00,
    /// ±500
    Range_500dps = 0b01,
    /// ±1000
    Range_1000dps = 0b10,
    /// ±2000
    Range_2000dps = 0b11,
}

//Gyro Full Scale Select: 00 = ±250dps
/// 01 = ±500dps
/// 10 = ±1000dps
/// 11 = ±2000dps

/// Sets default gyroscope range to ±2000
impl Default for GyroRange {
    fn default() -> Self {
        GyroRange::Range_2000dps
    }
}

impl GyroRange {
    /// convert degrees into radians
    const RADIANS_PER_DEGREE: f32 = core::f32::consts::PI / 180.0;

    /// Gyro range in radians per second per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::RADIANS_PER_DEGREE * self.resolution()
    }

    /// Gyro resolution in degrees per second per bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            GyroRange::Range_250dps => 250.0 / 32768.0,
            GyroRange::Range_500dps => 500.0 / 32768.0,
            GyroRange::Range_1000dps => 1000.0 / 32768.0,
            GyroRange::Range_2000dps => 2000.0 / 32768.0,
        }
    }
}

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
/// The accelerometer has a user-programmable accelerometer full-scale range
/// of ±2g, ±4g, ±8g, and ±16g.
/// (g is gravitational acceleration: 9.82 m/s^2)
/// The numeric values of these enums correspond to AFS_SEL
pub enum AccelRange {
    /// ±2g
    Range_2g = 0b00,
    /// ±4g
    Range_4g = 0b01,
    /// ±8g
    Range_8g = 0b10,
    /// ±16g
    Range_16g = 0b11,
}

impl Default for AccelRange {
    fn default() -> Self {
        AccelRange::Range_8g
    }
}

impl AccelRange {
    /// Earth gravitational acceleration (G) standard, in meters per second squared
    const EARTH_GRAVITY_ACCEL: f32 = 9.807;

    /// accelerometer scale in meters per second squared per bit
    pub(crate) fn scale(&self) -> f32 {
        Self::EARTH_GRAVITY_ACCEL * self.resolution()
    }

    /// Accelerometer resolution in G / bit
    /// Note that the ranges are ± which splits the raw i16 resolution between + and -
    pub(crate) fn resolution(&self) -> f32 {
        match self {
            Self::Range_2g => 2.0 / 32768.0,
            Self::Range_4g => 4.0 / 32768.0,
            Self::Range_8g => 8.0 / 32768.0,
            Self::Range_16g => 16.0 / 32768.0,
        }
    }
}
