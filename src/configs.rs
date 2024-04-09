use core::fmt::Debug;

use embedded_hal::{digital::OutputPin, spi::SpiDevice};

use crate::{CrcMode, DataRate, Device, Error, NRF24L01};

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug>
	NRF24L01<E, CE, SPI>
{
  /// Configure the nrf with sane settings
	pub fn configure(&mut self) -> Result<(), Error<SPIE>> {
		self.set_frequency(8)?;
		self.set_auto_retransmit(3, 7)?;
		self.set_rf(&DataRate::R250Kbps, 3)?;
		self.set_pipes_rx_enable(&[true, false, false, false, false, false])
			?;
		self.set_auto_ack(&[true, false, false, false, false, false])
			?;
		self.set_pipes_rx_lengths(&[None; 6])?;
		self.set_crc(CrcMode::TwoBytes)?;
		self.set_rx_addr(0, &b"fnord"[..]).unwrap();
		self.set_tx_addr(&b"fnord"[..])?;
		self.flush_rx()?;
		self.flush_tx()?;

		// Mask everything but rx data available
		self.update_config(|config| {
			config.set_mask_rx_dr(false);
			config.set_mask_tx_ds(true);
			config.set_mask_max_rt(true);
		})?;

    Ok(())
	}
}
