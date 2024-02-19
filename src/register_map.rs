#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum RegisterMap {
    VhvConfigTimeoutMacropLoopBound = 0x0008,
    VhvConfigInit = 0x000B,
    DefaultConfigBegin = 0x002D,
    GpioHvMuxCtrl = 0x0030,
    GpioTioHvStatus = 0x0031,
    SystemInterruptClear = 0x0086,
    SystemModeStart = 0x0087,
    SystemStatus = 0x00E5,
    ModelId = 0x010F,
    ModelType = 0x0110,
}
