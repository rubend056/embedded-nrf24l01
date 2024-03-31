use core::fmt::Debug;

use embedded_hal::{digital::OutputPin, spi::SpiDevice};

use crate::{CrcMode, DataRate, Error, NRF24L01};

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug>
	NRF24L01<E, CE, SPI>
{
  /// Initialize the nrf with sane settings
	pub fn init(&mut self) -> Result<(), Error<SPIE>> {
		self.set_frequency(8)?;
		self.set_auto_retransmit(15, 15).unwrap();
		self.set_rf(&DataRate::R250Kbps, 3).unwrap();
		self.set_pipes_rx_enable(&[true, false, false, false, false, false])
			.unwrap();
		self.set_auto_ack(&[true, false, false, false, false, false])
			.unwrap();
		self.set_pipes_rx_lengths(&[None; 6]).unwrap();
		self.set_crc(CrcMode::TwoBytes).unwrap();
		self.set_rx_addr(0, &b"fnord"[..]).unwrap();
		self.set_tx_addr(&b"fnord"[..])?;
		self.flush_rx()?;
		self.flush_tx()?;
    Ok(())
	}
}
