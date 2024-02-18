use embedded_hal::digital;
use embedded_hal::digital::{ErrorKind, ErrorType, OutputPin};

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NoXShutPinError {
    NotImplemented,
}

impl digital::Error for NoXShutPinError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

pub struct NoXShut {}

impl ErrorType for NoXShut {
    type Error = NoXShutPinError;
}

impl OutputPin for NoXShut {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Err(NoXShutPinError::NotImplemented)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Err(NoXShutPinError::NotImplemented)
    }
}
