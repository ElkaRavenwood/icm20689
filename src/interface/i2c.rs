use embedded_hal as hal;

use super::SensorInterface;
use crate::Error;

pub struct I2cInterface<I2C> {
    /// i2c port
    _i2c_port: I2C,
    /// address for i2c communications
    _address: u8,
}

impl<I2C, CommE> I2cInterface<I2C>
where
    I2C: hal::blocking::i2c::Read<Error = CommE>
        + hal::blocking::i2c::Write<Error = CommE>
        + hal::blocking::i2c::WriteRead<Error = CommE>,
{
    pub fn new(i2c: I2C, address: u8) -> Self {
        Self {
            _i2c_port: i2c,
            _address: address,
        }
    }
}

impl<I2C, CommE> SensorInterface for I2cInterface<I2C>
where
    I2C: hal::blocking::i2c::Read<Error = CommE>
        + hal::blocking::i2c::Write<Error = CommE>
        + hal::blocking::i2c::WriteRead<Error = CommE>,
{
    type InterfaceError = Error<CommE, ()>;

    fn register_read(&mut self, _reg: u8) -> Result<u8, Self::InterfaceError> {
        let mut buf = [0u8; 1];
        self._i2c_port.read(self._address, &mut buf[..]).map_err(|err| Error::Comm(err))?;
        Ok(buf[0])
    }

    fn register_write(&mut self, _reg: u8, _val: u8) -> Result<(), Self::InterfaceError> {
        let mut buf = [_val; 1];
        self._i2c_port.write( _reg, &mut buf[..]).map_err(|err| Error::Comm(err))?;
        Ok(())
    }

    fn read_vec3_i16(&mut self, _reg: u8) -> Result<[i16; 3], Self::InterfaceError> {
        unimplemented!()
    }

    fn using_spi(&self) -> bool {
        false
    }
}
