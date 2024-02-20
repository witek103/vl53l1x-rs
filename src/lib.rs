#![no_std]

mod default_configuration;
pub mod error;
pub mod no_x_shut;
pub mod register_map;

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::i2c::{I2c, SevenBitAddress};
use no_x_shut::NoXShut;
use register_map::{DistanceMode, RegisterMap, TimingBudget};

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
    pub async fn read_register_word(
        &mut self,
        register: RegisterMap,
    ) -> Result<u16, Error<I2C::Error>> {
        let mut buf = [0; 2];

        self.dev
            .write_read(self.address, &(register as u16).to_be_bytes(), &mut buf)
            .await?;

        Ok(u16::from_be_bytes(buf))
    }

    pub async fn write_register(
        &mut self,
        register: RegisterMap,
        value: u8,
    ) -> Result<(), Error<I2C::Error>> {
        let dest = (register as u16).to_be_bytes();

        self.dev
            .write(self.address, &[dest[0], dest[1], value])
            .await?;

        Ok(())
    }

    pub async fn write_register_u16(
        &mut self,
        register: RegisterMap,
        value: u16,
    ) -> Result<(), Error<I2C::Error>> {
        let dest = (register as u16).to_be_bytes();
        let value = value.to_be_bytes();

        self.dev
            .write(self.address, &[dest[0], dest[1], value[0], value[1]])
            .await?;

        Ok(())
    }

    pub async fn write_register_u32(
        &mut self,
        register: RegisterMap,
        value: u32,
    ) -> Result<(), Error<I2C::Error>> {
        let dest = (register as u16).to_be_bytes();
        let value = value.to_be_bytes();

        self.dev
            .write(
                self.address,
                &[dest[0], dest[1], value[0], value[1], value[2], value[3]],
            )
            .await?;

        Ok(())
    }

    pub async fn write_multiple(
        &mut self,
        register_begin: RegisterMap,
        buf: &[u8],
    ) -> Result<(), Error<I2C::Error>> {
        let dest_begin = register_begin as u16;

        for (offset, value) in buf.iter().enumerate() {
            let dest = (dest_begin + offset as u16).to_be_bytes();

            self.dev
                .write(self.address, &[dest[0], dest[1], *value])
                .await?
        }

        Ok(())
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

    /// Start continuous ranging
    pub async fn start_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("Start ranging on sensor at {}", self.address);

        self.write_register(RegisterMap::SystemModeStart, 0x40)
            .await
    }

    pub async fn stop_ranging(&mut self) -> Result<(), Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("Stop ranging on sensor at {}", self.address);

        self.write_register(RegisterMap::SystemModeStart, 0x00)
            .await
    }

    /// Must be called after reading ranging data in order to trigger next measurement
    pub async fn clear_interrupt(&mut self) -> Result<(), Error<I2C::Error>> {
        self.write_register(RegisterMap::SystemInterruptClear, 0x01)
            .await
    }

    /// Poll for pending ranging data
    pub async fn data_ready(&mut self) -> Result<bool, Error<I2C::Error>> {
        let gpio_hv_mux_ctrl = self.read_register(RegisterMap::GpioHvMuxCtrl).await?;

        let interrupt_polarity_low = gpio_hv_mux_ctrl & (1 << 4) == 1 << 4;

        let gpio_tio_hv_status = self.read_register(RegisterMap::GpioTioHvStatus).await?;

        let data_ready_bit_low = gpio_tio_hv_status & 0x01 == 0x00;

        Ok(interrupt_polarity_low == data_ready_bit_low)
    }

    pub async fn init(
        &mut self,
        delay: &mut impl DelayNs,
        timeout_ms: u32,
    ) -> Result<(), Error<I2C::Error>> {
        #[cfg(feature = "defmt")]
        defmt::trace!("Initializing sensor at {}", self.address);

        self.write_multiple(
            RegisterMap::DefaultConfigBegin,
            &default_configuration::DEFAULT_CONFIGURATION,
        )
        .await?;

        self.start_ranging().await?;

        let mut time_passed_ms = 0;

        loop {
            delay.delay_ms(1).await;

            time_passed_ms += 1;

            if self.data_ready().await? {
                break;
            }

            if time_passed_ms > timeout_ms {
                return Err(Error::Timout);
            }
        }

        self.clear_interrupt().await?;
        self.stop_ranging().await?;

        self.write_register(RegisterMap::VhvConfigTimeoutMacropLoopBound, 0x09)
            .await?;
        self.write_register(RegisterMap::VhvConfigInit, 0x00)
            .await?;

        #[cfg(feature = "defmt")]
        defmt::trace!("Sensor at {} initialized", self.address);

        Ok(())
    }

    pub async fn set_interrupt_polarity(
        &mut self,
        active_high: bool,
    ) -> Result<(), Error<I2C::Error>> {
        let gpio_hv_mux_ctrl = self.read_register(RegisterMap::GpioHvMuxCtrl).await?;

        let polarity_bit = if active_high { 0 } else { 1 };

        self.write_register(
            RegisterMap::GpioHvMuxCtrl,
            (gpio_hv_mux_ctrl & 0xEF) | (polarity_bit << 4),
        )
        .await
    }

    pub async fn get_timing_budget(&mut self) -> Result<TimingBudget, Error<I2C::Error>> {
        let distance_mode = self.get_distance_mode().await?;

        let timeout_macrop_a_hi = self
            .read_register_word(RegisterMap::RangeConfigTimeoutMacropAHi)
            .await?;

        let timing_budget =
            TimingBudget::from_timeout_macrop_a_hi(&distance_mode, timeout_macrop_a_hi)
                .map_err(|_| Error::InvalidTimingBudget)?;

        Ok(timing_budget)
    }

    pub async fn set_timing_budget(
        &mut self,
        timing_budget: TimingBudget,
    ) -> Result<(), Error<I2C::Error>> {
        let distance_mode = self.get_distance_mode().await?;

        let timeout_macrop_a_hi = timing_budget
            .get_timeout_macrop_a_hi(&distance_mode)
            .map_err(|_| Error::InvalidTimingBudget)?;
        let timeout_macrop_b_hi = timing_budget
            .get_timeout_macrop_b_hi(&distance_mode)
            .map_err(|_| Error::InvalidTimingBudget)?;

        self.write_register_u16(
            RegisterMap::RangeConfigTimeoutMacropAHi,
            timeout_macrop_a_hi,
        )
        .await?;
        self.write_register_u16(
            RegisterMap::RangeConfigTimeoutMacropBHi,
            timeout_macrop_b_hi,
        )
        .await
    }

    pub async fn set_distance_mode(
        &mut self,
        distance_mode: DistanceMode,
    ) -> Result<(), Error<I2C::Error>> {
        let timing_budget = self.get_timing_budget().await?;

        let (
            timeout_macrop,
            vcsel_period_a,
            vcsel_period_b,
            valid_phase_high,
            woi_sd0,
            initial_phase_sd0,
        ) = match distance_mode {
            DistanceMode::Short => (0x14, 0x07, 0x05, 0x38, 0x0705, 0x0606),
            DistanceMode::Long => (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),
        };

        self.write_register(RegisterMap::PhasecalConfigTimeoutMacrop, timeout_macrop)
            .await?;
        self.write_register(RegisterMap::RangeConfigVcselPeriodA, vcsel_period_a)
            .await?;
        self.write_register(RegisterMap::RangeConfigVcselPeriodB, vcsel_period_b)
            .await?;
        self.write_register(RegisterMap::RangeConfigValidPhaseHigh, valid_phase_high)
            .await?;
        self.write_register_u16(RegisterMap::SdConfigWoiSd0, woi_sd0)
            .await?;
        self.write_register_u16(RegisterMap::SdConfigInitialPhaseSd0, initial_phase_sd0)
            .await?;

        self.set_timing_budget(timing_budget).await
    }

    pub async fn get_distance_mode(&mut self) -> Result<DistanceMode, Error<I2C::Error>> {
        let timeout_macrop = self
            .read_register(RegisterMap::PhasecalConfigTimeoutMacrop)
            .await?;

        let distance_mode = match timeout_macrop {
            0x14 => DistanceMode::Short,
            0x0A => DistanceMode::Long,
            _ => return Err(Error::InvalidDistanceMode),
        };

        Ok(distance_mode)
    }

    /// Caution: period must be equal or less than timing budget
    pub async fn set_inter_measurement_period(
        &mut self,
        period_ms: u16,
    ) -> Result<(), Error<I2C::Error>> {
        let clock_pll = self
            .read_register_word(RegisterMap::ResultOscCalibrateVal)
            .await?;

        let period = (clock_pll & 0x3FF) as f64 * period_ms as f64 * 1.075;

        self.write_register_u32(RegisterMap::SystemIntermeasurementPeriod, period as u32)
            .await
    }
}
