#![no_std]

pub mod error;
pub mod no_x_shut;
pub mod register_map;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use no_x_shut::NoXShut;
use register_map::RegisterMap;

pub use error::Error;

const I2C_ADDR_DEFAULT: SevenBitAddress = 0x29;

const MODEL_ID: u8 = 0xEA;
const MODEL_TYPE: u8 = 0xCC;

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
    pub async fn read_register(&mut self, register: RegisterMap) -> Result<u8, Error<I2C::Error>> {
        let mut buf = [0; 1];

        self.dev
            .write_read(self.address, &(register as u16).to_be_bytes(), &mut buf)
            .await?;

        Ok(buf[0])
    }

    pub async fn get_model_id(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(RegisterMap::ModelId).await
    }

    pub async fn get_model_type(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(RegisterMap::ModelType).await
    }

    /// Bit 0 is low when device is not yet booted, high after boot.
    pub async fn get_boot_state(&mut self) -> Result<u8, Error<I2C::Error>> {
        self.read_register(RegisterMap::SystemStatus).await
    }

    pub fn disable(&mut self) -> Result<(), Error<I2C::Error>> {
        self.x_shut.set_low().map_err(|_| Error::DisableFailed)
    }

    pub fn enable(&mut self) -> Result<(), Error<I2C::Error>> {
        self.x_shut.set_high().map_err(|_| Error::EnableFailed)
    }

    /// Perform reset using XShut pin, returns time spent in millis.
    pub async fn reset(&mut self, delay: &mut impl DelayNs) -> Result<u32, Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("Resetting sensor at {}", self.address);

        self.disable()?;

        delay.delay_ms(2).await;

        self.enable()?;

        delay.delay_ms(2).await;

        Ok(4)
    }

    /// Reset sensor until it is detected or timout occurs.
    /// Verifies model ID and type and ensures communication works.
    pub async fn reset_until_detection(
        &mut self,
        delay: &mut impl DelayNs,
        timeout_ms: u32,
    ) -> Result<(), Error<I2C::Error>> {
        let mut time_passed_ms = 0;

        let model_id = loop {
            time_passed_ms += self.reset(delay).await?;

            if let Ok(model_id) = self.get_model_id().await {
                break model_id;
            }

            if time_passed_ms > timeout_ms {
                return Err(Error::Timout);
            }
        };

        if model_id != MODEL_ID || self.get_model_type().await? != MODEL_TYPE {
            return Err(Error::WrongSensorModel);
        }

        Ok(())
    }

    /// Wait until device is fully booted or timeout occurs.
    pub async fn wait_until_booted(
        &mut self,
        delay: &mut impl DelayNs,
        timeout_ms: u32,
    ) -> Result<(), Error<I2C::Error>> {
        let mut time_passed_ms = 0;

        loop {
            delay.delay_ms(1).await;

            time_passed_ms += 1;

            let boot_state = self.get_boot_state().await?;

            if boot_state & 0x01 == 0x01 {
                break;
            }

            if time_passed_ms > timeout_ms {
                return Err(Error::Timout);
            }
        }

        #[cfg(feature = "defmt")]
        defmt::trace!("Sensor at {} booted correctly", self.address);

        Ok(())
    }
}
