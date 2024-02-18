#[repr(u16)]
pub enum RegisterMap {
    SystemStatus = 0x00E5,
    ModelId = 0x010F,
    ModelType = 0x0110,
}
