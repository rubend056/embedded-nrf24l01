use core::fmt::Debug;

/// Wraps an SPI error
///
/// TODO: eliminate this?
#[derive(Debug)]
pub enum Error<SPIE: Debug> {
    /// Wrap an SPI error
    SpiError(SPIE),
    /// Module not connected
    NotConnected,
    /// Module not connected
    PayloadSizeInvalid,
    /// The tx fifo is full
    TxFifoFull,
}

impl<SPIE: Debug> From<SPIE> for Error<SPIE> {
    fn from(e: SPIE) -> Self {
        Error::SpiError(e)
    }
}
