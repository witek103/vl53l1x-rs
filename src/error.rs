#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<T> {
    I2CError(T),
    Other,
}

impl<T> From<T> for Error<T> {
    fn from(value: T) -> Self {
        Self::I2CError(value)
    }
}
