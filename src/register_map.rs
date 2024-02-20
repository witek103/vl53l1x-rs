#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum RegisterMap {
    VhvConfigTimeoutMacropLoopBound = 0x0008,
    VhvConfigInit = 0x000B,
    DefaultConfigBegin = 0x002D,
    GpioHvMuxCtrl = 0x0030,
    GpioTioHvStatus = 0x0031,
    PhasecalConfigTimeoutMacrop = 0x004B,
    RangeConfigTimeoutMacropAHi = 0x005E,
    RangeConfigVcselPeriodA = 0x0060,
    RangeConfigTimeoutMacropBHi = 0x0061,
    RangeConfigVcselPeriodB = 0x0063,
    RangeConfigValidPhaseHigh = 0x0069,
    SystemIntermeasurementPeriod = 0x006C,
    SdConfigWoiSd0 = 0x0078,
    SdConfigInitialPhaseSd0 = 0x007A,
    SystemInterruptClear = 0x0086,
    SystemModeStart = 0x0087,
    ResultFinalCrosstalkCorrectedRangeMmSd0 = 0x0096,
    ResultOscCalibrateVal = 0x00DE,
    SystemStatus = 0x00E5,
    ModelId = 0x010F,
    ModelType = 0x0110,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DistanceMode {
    Short,
    Long,
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TimingBudget {
    Ms15,
    Ms20,
    Ms33,
    Ms50,
    Ms100,
    Ms200,
    Ms500,
}

impl TimingBudget {
    pub fn from_timeout_macrop_a_hi(
        distance_mode: &DistanceMode,
        timeout_macrop_a_hi: u16,
    ) -> Result<Self, ()> {
        let timing_budget = match distance_mode {
            DistanceMode::Short => match timeout_macrop_a_hi {
                0x001D => TimingBudget::Ms15,
                0x0051 => TimingBudget::Ms20,
                0x00D6 => TimingBudget::Ms33,
                0x01AE => TimingBudget::Ms50,
                0x02E1 => TimingBudget::Ms100,
                0x03E1 => TimingBudget::Ms200,
                0x0591 => TimingBudget::Ms500,
                _ => return Err(()),
            },
            DistanceMode::Long => match timeout_macrop_a_hi {
                0x001E => TimingBudget::Ms20,
                0x0060 => TimingBudget::Ms33,
                0x00AD => TimingBudget::Ms50,
                0x01CC => TimingBudget::Ms100,
                0x02D9 => TimingBudget::Ms200,
                0x048F => TimingBudget::Ms500,
                _ => return Err(()),
            },
        };

        Ok(timing_budget)
    }

    pub fn get_timeout_macrop_a_hi(&self, distance_mode: &DistanceMode) -> Result<u16, ()> {
        let timeout_macrop_a_hi = match distance_mode {
            DistanceMode::Short => match self {
                TimingBudget::Ms15 => 0x001D,
                TimingBudget::Ms20 => 0x0051,
                TimingBudget::Ms33 => 0x00D6,
                TimingBudget::Ms50 => 0x01AE,
                TimingBudget::Ms100 => 0x02E1,
                TimingBudget::Ms200 => 0x03E1,
                TimingBudget::Ms500 => 0x0591,
            },
            DistanceMode::Long => match self {
                TimingBudget::Ms20 => 0x001E,
                TimingBudget::Ms33 => 0x0060,
                TimingBudget::Ms50 => 0x00AD,
                TimingBudget::Ms100 => 0x01CC,
                TimingBudget::Ms200 => 0x02D9,
                TimingBudget::Ms500 => 0x048F,
                _ => return Err(()),
            },
        };

        Ok(timeout_macrop_a_hi)
    }

    pub fn get_timeout_macrop_b_hi(&self, distance_mode: &DistanceMode) -> Result<u16, ()> {
        let timeout_macrop_b_hi = match distance_mode {
            DistanceMode::Short => match self {
                TimingBudget::Ms15 => 0x0027,
                TimingBudget::Ms20 => 0x006E,
                TimingBudget::Ms33 => 0x006E,
                TimingBudget::Ms50 => 0x01E8,
                TimingBudget::Ms100 => 0x0388,
                TimingBudget::Ms200 => 0x0496,
                TimingBudget::Ms500 => 0x05C1,
            },
            DistanceMode::Long => match self {
                TimingBudget::Ms20 => 0x0022,
                TimingBudget::Ms33 => 0x006E,
                TimingBudget::Ms50 => 0x00C6,
                TimingBudget::Ms100 => 0x01EA,
                TimingBudget::Ms200 => 0x02F8,
                TimingBudget::Ms500 => 0x04A4,
                _ => return Err(()),
            },
        };

        Ok(timeout_macrop_b_hi)
    }
}
