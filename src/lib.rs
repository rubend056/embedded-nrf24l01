// Copyright 2018, Astro <astro@spaceboyz.net>
//
// Licensed under the Apache License, Version 2.0 <LICENSE>. This file
// may not be copied, modified, or distributed except according to
// those terms.

//! nRF24L01+ driver for use with [embedded-hal](https://crates.io/crates/embedded-hal)

#![warn(missing_docs, unused)]


#![no_std]
#[macro_use]
extern crate bitfield;

use core::fmt;
use core::fmt::Debug;
use embedded_hal::spi::SpiDevice;
use embedded_hal::digital::OutputPin;
use heapless::Vec;

mod config;
pub use crate::config::{Configuration, CrcMode, DataRate};
pub mod setup;

mod registers;
use crate::registers::{Config, Register, SetupAw, Status, Feature};
mod command;
use crate::command::{Command, ReadRegister, WriteRegister};
mod error;
pub use crate::error::Error;

mod device;
pub use crate::device::Device;
mod standby;
pub use crate::standby::StandbyMode;
mod rx;
pub use crate::rx::RxMode;
mod tx;
pub use crate::tx::TxMode;

/// Number of RX pipes with configurable addresses
pub const PIPES_COUNT: usize = 6;
/// Minimum address length
pub const MIN_ADDR_BYTES: usize = 2;
/// Maximum address length
pub const MAX_ADDR_BYTES: usize = 5;

/// Driver for the nRF24L01+
///
/// Never deal with this directly. Instead, you store one of the following types:
///
/// * [`StandbyMode<D>`](struct.StandbyMode.html)
/// * [`RxMode<D>`](struct.RxMode.html)
/// * [`TxMode<D>`](struct.TxMode.html)
///
/// where `D: `[`Device`](trait.Device.html)
pub struct NRF24L01<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8>> {
    ce: CE,
    pub spi: SPI,
    config: Config,
}

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug> fmt::Debug
    for NRF24L01<E, CE, SPI>
{
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "NRF24L01")
    }
}

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug>
    NRF24L01<E, CE, SPI>
{
    /// Construct a new driver instance.
    pub fn new(mut ce: CE, spi: SPI) -> Result<StandbyMode<Self>, Error<SPIE>> {
        ce.set_low().unwrap();

        // Reset value
        let mut config = Config(0b0000_1000);
        config.set_mask_rx_dr(false);
        config.set_mask_tx_ds(false);
        config.set_mask_max_rt(false);
        let mut device = NRF24L01 {
            ce,
            spi,
            config,
        };

        match device.is_connected() {
            Err(e) => return Err(e),
            Ok(false) => return Err(Error::NotConnected),
            _ => {}
        }

        // Enable features
        let mut features = Feature(0);
        features.set_en_dyn_ack(true);
        features.set_en_dpl(true);
        device.write_register(features)?;

        StandbyMode::power_up(device).map_err(|(_, e)| e)
    }

    /// Reads and validates content of the `SETUP_AW` register.
    pub fn is_connected(&mut self) -> Result<bool, Error<SPIE>> {
        let (_, setup_aw) = self.read_register::<SetupAw>()?;
        let valid = setup_aw.aw() <= 3;
        Ok(valid)
    }
}

impl<E: Debug, CE: OutputPin<Error = E>, SPI: SpiDevice<u8, Error = SPIE>, SPIE: Debug> Device
    for NRF24L01<E, CE, SPI>
{
    type Error = Error<SPIE>;

    fn ce_enable(&mut self) {
        self.ce.set_high().unwrap();
    }

    fn ce_disable(&mut self) {
        self.ce.set_low().unwrap();
    }

    fn send_command<C: Command>(
        &mut self,
        command: &C,
    ) -> Result<(Status, C::Response), Self::Error> {

        // Allocate storage
        let mut v = Vec::<u8, 33>::new();
        v.resize_default(command.len()).unwrap();
        command.encode(&mut v);
        
        // SPI transaction
        self.spi.transfer_in_place(&mut v)?;

        // Parse response
        let status = Status(v[0]);
        let response = C::decode_response(v);

        Ok((status, response))
    }

    fn write_register<R: Register>(&mut self, register: R) -> Result<Status, Self::Error> {
        let (status, ()) = self.send_command(&WriteRegister::new(register))?;
        Ok(status)
    }

    fn read_register<R: Register>(&mut self) -> Result<(Status, R), Self::Error> {
        self.send_command(&ReadRegister::new())
    }

    fn update_config<F, R>(&mut self, f: F) -> Result<R, Self::Error>
    where
        F: FnOnce(&mut Config) -> R,
    {
        // Mutate
        let old_config = self.config.clone();
        let result = f(&mut self.config);

        if self.config != old_config {
            let config = self.config.clone();
            self.write_register(config)?;
        }
        Ok(result)
    }
}
