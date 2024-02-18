#![no_std]

pub mod no_x_shut;
pub mod register_map;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use no_x_shut::NoXShut;
use register_map::RegisterMap;

const I2C_ADDR_DEFAULT: SevenBitAddress = 0x29;

pub struct Vl53l1x<I2C, XShut> {
    dev: I2C,
    x_shut: XShut,
    address: SevenBitAddress,
}

impl<I2C> Vl53l1x<I2C, NoXShut>
where
    I2C: I2c,
{
    /// Use driver with default I2C address without XShut pin support
    pub fn new(dev: I2C) -> Self {
        Self {
            dev,
            x_shut: NoXShut {},
            address: I2C_ADDR_DEFAULT,
        }
    }

    /// Assign XShut pin to control software shutdown
    pub fn with_x_shut<XShut>(self, x_shut: XShut) -> Vl53l1x<I2C, XShut> {
        Vl53l1x {
            dev: self.dev,
            x_shut,
            address: self.address,
        }
    }
}

impl<I2C, XShut> Vl53l1x<I2C, XShut>
where
    I2C: I2c,
    XShut: OutputPin,
{
    pub async fn read_register(&mut self, register: RegisterMap) -> Result<u8, I2C::Error> {
        let mut buf = [0; 1];

        self.dev
            .write_read(self.address, &(register as u16).to_be_bytes(), &mut buf)
            .await?;

        Ok(buf[0])
    }

    pub async fn get_model_id(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(RegisterMap::ModelId).await
    }

    pub async fn get_model_type(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(RegisterMap::ModelType).await
    }

    /// Bit 0 is low when device is not yet booted, high after boot.
    pub async fn get_boot_state(&mut self) -> Result<u8, I2C::Error> {
        self.read_register(RegisterMap::SystemStatus).await
    }
}
