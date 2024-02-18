#![no_std]

pub mod register_map;

use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use register_map::RegisterMap;

const I2C_ADDR_DEFAULT: SevenBitAddress = 0x29;

pub struct Vl53l1x<T> {
    dev: T,
    address: SevenBitAddress,
}

impl<T> Vl53l1x<T>
where
    T: I2c,
{
    /// Use driver with default I2C address
    pub fn new(dev: T) -> Self {
        Self {
            dev,
            address: I2C_ADDR_DEFAULT,
        }
    }

    pub async fn read_register(&mut self, register: RegisterMap) -> Result<u8, T::Error> {
        let mut buf = [0; 1];

        self.dev
            .write_read(self.address, &(register as u16).to_be_bytes(), &mut buf)
            .await?;

        Ok(buf[0])
    }

    pub async fn get_model_id(&mut self) -> Result<u8, T::Error> {
        self.read_register(RegisterMap::ModelId).await
    }

    pub async fn get_model_type(&mut self) -> Result<u8, T::Error> {
        self.read_register(RegisterMap::ModelType).await
    }

    /// Bit 0 is low when device is not yet booted, high after boot.
    pub async fn get_boot_state(&mut self) -> Result<u8, T::Error> {
        self.read_register(RegisterMap::SystemStatus).await
    }
}
