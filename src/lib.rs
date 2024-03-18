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

use command::{FlushRx, FlushTx, Nop, ReadRxPayload, ReadRxPayloadWidth, WriteTxPayload};
use core::fmt;
use core::fmt::Debug;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;
use heapless::Vec;
use registers::{EnAa, EnRxaddr, FifoStatus, ObserveTx, RfCh, SetupRetr, TxAddr, CD};

pub mod setup;

mod registers;
use crate::registers::{Config, Dynpd, Feature, Register, RfSetup, SetupAw, Status};
mod command;
use crate::command::{Command, ReadRegister, WriteRegister};
mod error;
pub use crate::error::Error;

pub mod device;
pub use crate::device::Device;

/// Number of RX pipes with configurable addresses
pub const PIPES_COUNT: usize = 6;
/// Minimum address length
pub const MIN_ADDR_BYTES: usize = 2;
/// Maximum address length
pub const MAX_ADDR_BYTES: usize = 5;

/// Supported air data rates.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum DataRate {
    /// 250 Kbps
    R250Kbps,
    /// 1 Mbps
    R1Mbps,
    /// 2 Mbps
    R2Mbps,
}

impl Default for DataRate {
    fn default() -> DataRate {
        DataRate::R1Mbps
    }
}

/// Supported CRC modes
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum CrcMode {
    /// Disable all CRC generation/checking
    Disabled,
    /// One byte checksum
    OneByte,
    /// Two bytes checksum
    TwoBytes,
}

impl CrcMode {
    fn set_config(&self, config: &mut Config) {
        let (en_crc, crco) = match *self {
            CrcMode::Disabled => (false, false),
            CrcMode::OneByte => (true, false),
            CrcMode::TwoBytes => (true, true),
        };
        config.set_en_crc(en_crc);
        config.set_crco(crco);
    }
}

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
    pub fn new(mut ce: CE, spi: SPI) -> Result<Self, Error<SPIE>> {
        ce.set_low().unwrap();

        // Reset value
        let mut config = Config(0b0000_1000);
        config.set_mask_rx_dr(false);
        config.set_mask_tx_ds(false);
        config.set_mask_max_rt(false);
        let mut device = NRF24L01 { ce, spi, config };
        

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

        Ok(device)
    }

    /// Reads and validates content of the `SETUP_AW` register.
    pub fn is_connected(&mut self) -> Result<bool, Error<SPIE>> {
        let (_, setup_aw) = self.read_register::<SetupAw>()?;
        let valid = setup_aw.aw() <= 3;
        Ok(valid)
    }
    /// Powers up the device
    pub fn power_up(mut self) -> Result<(), Error<SPIE>> {
        self.update_config(|config| config.set_pwr_up(true))
    }

    /// Should be a no-op
    pub fn power_down(mut self) -> Result<(), Error<SPIE>> {
        self.update_config(|config| config.set_pwr_up(false))
    }
    /// Sets device as primary rx, enables chip
    pub fn rx(&mut self) -> Result<(), Error<SPIE>> {
        self.update_config(|config| config.set_prim_rx(true))?;
        self.ce_enable();
        Ok(())
    }
    /// Sets device as primary tx
    pub fn tx(&mut self) -> Result<(), Error<SPIE>> {
        self.update_config(|config| config.set_prim_rx(false))
    }

    /// Is there any incoming data to read? Return the pipe number.
    ///
    /// This function acknowledges all interrupts even if there are more received packets, so the
    /// caller must repeat the call until the function returns None before waiting for the next RX
    /// interrupt.
    pub fn can_read(&mut self) -> Result<Option<u8>, Error<SPIE>> {
        // Acknowledge all interrupts.
        // Note that we cannot selectively acknowledge the RX interrupt here - if any TX interrupt
        // is still active, the IRQ pin could otherwise not be used for RX interrupts.
        let mut clear = Status(0);
        clear.set_rx_dr(true);
        clear.set_tx_ds(true);
        clear.set_max_rt(true);
        self.write_register(clear)?;

        self.read_register::<FifoStatus>()
            .map(|(status, fifo_status)| {
                if !fifo_status.rx_empty() {
                    Some(status.rx_p_no())
                } else {
                    None
                }
            })
    }

    /// Is an in-band RF signal detected?
    ///
    /// The internal carrier detect signal must be high for 40μs
    /// (NRF24L01+) or 128μs (NRF24L01) before the carrier detect
    /// register is set. Note that changing from standby to receive
    /// mode also takes 130μs.
    pub fn has_carrier(&mut self) -> Result<bool, Error<SPIE>> {
        self.read_register::<CD>().map(|(_, cd)| cd.0 & 1 == 1)
    }

    /// Is the RX queue empty?
    pub fn is_rx_empty(&mut self) -> Result<bool, Error<SPIE>> {
        self.read_register::<FifoStatus>()
            .map(|(_, fifo_status)| fifo_status.rx_empty())
    }

    /// Is the RX queue full?
    pub fn is_rx_full(&mut self) -> Result<bool, Error<SPIE>> {
        self.read_register::<FifoStatus>()
            .map(|(_, fifo_status)| fifo_status.rx_full())
    }

    /// Read the next packet, blocking
    pub fn read(&mut self) -> Result<Vec<u8, 33>, Error<SPIE>> {
        let (_, payload_width) = self.send_command(&ReadRxPayloadWidth)?;
        let (_, payload) = self.send_command(&ReadRxPayload::new(payload_width as usize))?;
        Ok(payload)
    }

    /// Receive a packet, non-blocking
    pub fn receive(&mut self) -> nb::Result<Vec<u8, 33>, Error<SPIE>> {
        if self.can_read()?.is_some() {
            nb::Result::Ok(self.read()?)
        } else {
            nb::Result::Err(nb::Error::WouldBlock)
        }
    }


    /// Is TX FIFO empty?
    pub fn is_tx_empty(&mut self) -> Result<bool, Error<SPIE>> {
        let (_, fifo_status) = self.read_register::<FifoStatus>()?;
        Ok(fifo_status.tx_empty())
    }

    /// Is TX FIFO full?
    pub fn is_tx_full(&mut self) -> Result<bool, Error<SPIE>> {
        let (_, fifo_status) = self.read_register::<FifoStatus>()?;
        Ok(fifo_status.tx_full())
    }

    /// Does the TX FIFO have space?
    pub fn can_send(&mut self) -> Result<bool, Error<SPIE>> {
        let full = self.is_tx_full()?;
        Ok(!full)
    }

    /// Put payload in TX FIFO and start transmission.
    pub fn send(&mut self, packet: &[u8]) -> Result<(), Error<SPIE>> {
        self.send_command(&WriteTxPayload::new(packet))?;
        self.ce_enable();
        Ok(())
    }

    /// Poll completion of one or multiple send operations and check whether transmission was
    /// successful.
    ///
    /// This function behaves like `wait_empty()`, except that it returns whether sending was
    /// successful and that it provides an asynchronous interface.
    ///
    /// Automatic retransmission (set_auto_retransmit) and acks (set_auto_ack) have to be
    /// enabled if you actually want to know if transmission was successful. 
    /// Else the nrf24 just transmits the packet once and assumes it was received.
    pub fn poll_send(&mut self) -> nb::Result<bool, Error<SPIE>> {
        let (status, fifo_status) = self.read_register::<FifoStatus>()?;
        // We need to clear all the TX interrupts whenever we return Ok here so that the next call
        // to poll_send correctly recognizes max_rt and send completion.
        if status.max_rt() {
            // If MAX_RT is set, the packet is not removed from the FIFO, so if we do not flush
            // the FIFO, we end up in an infinite loop
            self.send_command(&FlushTx)?;
            self.clear_interrupts_and_ce()?;
            Ok(false)
        } else if fifo_status.tx_empty() {
            self.clear_interrupts_and_ce()?;
            Ok(true)
        } else {
            self.ce_enable();
            Err(nb::Error::WouldBlock)
        }
    }

    fn clear_interrupts_and_ce(&mut self) -> nb::Result<(), Error<SPIE>> {
        let mut clear = Status(0);
        clear.set_tx_ds(true);
        clear.set_max_rt(true);
        self.write_register(clear)?;

        // Can save power now
        self.ce_disable();

        Ok(())
    }
    /// Read the `OBSERVE_TX` register
    pub fn observe_tx(&mut self) -> Result<ObserveTx, Error<SPIE>> {
        let (_, observe_tx) = self.read_register()?;
        Ok(observe_tx)
    }


    //Config

    /// Flush RX queue
    ///
    /// Discards all received packets that have not yet been [read](struct.RxMode.html#method.read) from the RX FIFO
    pub fn flush_rx(&mut self) -> Result<(), Error<SPIE>> {
        self.send_command(&FlushRx)?;
        Ok(())
    }

    /// Flush TX queue, discarding any unsent packets
    pub fn flush_tx(&mut self) -> Result<(), Error<SPIE>> {
        self.send_command(&FlushTx)?;
        Ok(())
    }

    /// Get frequency offset (channel)
    pub fn get_frequency(&mut self) -> Result<u8, Error<SPIE>> {
        let (_, register) = self.read_register::<RfCh>()?;
        let freq_offset = register.rf_ch();
        Ok(freq_offset)
    }

    /// Set frequency offset (channel)
    pub fn set_frequency(
        &mut self,
        freq_offset: u8,
    ) -> Result<(), Error<SPIE>> {
        assert!(freq_offset < 126);

        let mut register = RfCh(0);
        register.set_rf_ch(freq_offset);
        self.write_register(register)?;

        Ok(())
    }

    /// power: `0`: -18 dBm, `3`: 0 dBm
    pub fn set_rf(
        &mut self,
        rate: &DataRate,
        power: u8,
    ) -> Result<(), Error<SPIE>> {
        assert!(power < 0b100);
        let mut register = RfSetup(0);
        register.set_rf_pwr(power);

        let (dr_low, dr_high) = match *rate {
            DataRate::R250Kbps => (true, false),
            DataRate::R1Mbps => (false, false),
            DataRate::R2Mbps => (false, true),
        };
        register.set_rf_dr_low(dr_low);
        register.set_rf_dr_high(dr_high);

        self.write_register(register)?;
        Ok(())
    }

    /// Set CRC mode
    pub fn set_crc(
        &mut self,
        mode: CrcMode,
    ) -> Result<(), Error<SPIE>> {
        self.update_config(|config| mode.set_config(config))
    }

    /// Sets the interrupt mask
    /// 
    /// When an interrupt mask is set to true, the interrupt is masked and will not fire on the IRQ pin.
    /// When set to false, it will trigger the IRQ pin.
    pub fn set_interrupt_mask(
        &mut self,
        data_ready_rx: bool,
        data_sent_tx: bool,
        max_retransmits_tx: bool
    ) -> Result<(), Error<SPIE>> {
        self.update_config(|config| {
            config.set_mask_rx_dr(data_ready_rx);
            config.set_mask_tx_ds(data_sent_tx);
            config.set_mask_max_rt(max_retransmits_tx);
        })
    }

    /// Configure which RX pipes to enable
    pub fn set_pipes_rx_enable(
        &mut self,
        bools: &[bool; PIPES_COUNT],
    ) -> Result<(), Error<SPIE>> {
        self.write_register(EnRxaddr::from_bools(bools))?;
        Ok(())
    }

    /// Set address `addr` of pipe number `pipe_no`
    pub fn set_rx_addr(
        &mut self,
        pipe_no: usize,
        addr: &[u8],
    ) -> Result<(), Error<SPIE>> {
        macro_rules! w {
            ( $($no: expr, $name: ident);+ ) => (
                match pipe_no {
                    $(
                        $no => {
                            use crate::registers::$name;
                            let register = $name::new(addr);
                            self.write_register(register)?;
                        }
                    )+
                        _ => panic!("No such pipe {}", pipe_no)
                }
            )
        }
        w!(0, RxAddrP0;
           1, RxAddrP1;
           2, RxAddrP2;
           3, RxAddrP3;
           4, RxAddrP4;
           5, RxAddrP5);
        Ok(())
    }

    /// Set address of the TX pipe
    pub fn set_tx_addr(
        &mut self,
        addr: &[u8],
    ) -> Result<(), Error<SPIE>> {
        let register = TxAddr::new(addr);
        self.write_register(register)?;
        Ok(())
    }

    /// Configure auto-retransmit
    ///
    /// To disable, call as `set_auto_retransmit(0, 0)`.
    pub fn set_auto_retransmit(
        &mut self,
        delay: u8,
        count: u8,
    ) -> Result<(), Error<SPIE>> {
        let mut register = SetupRetr(0);
        register.set_ard(delay);
        register.set_arc(count);
        self.write_register(register)?;
        Ok(())
    }

    /// Obtain auto-acknowledgment configuration for all pipes
    pub fn get_auto_ack(
        &mut self,
    ) -> Result<[bool; PIPES_COUNT], Error<SPIE>> {
        // Read
        let (_, register) = self.read_register::<EnAa>()?;
        Ok(register.to_bools())
    }

    /// Configure auto-acknowledgment for all RX pipes
    ///
    /// Auto ack is handled by the nrf24 if:
    /// 1. Auto ack feature is enabled on Feature Register
    /// 2. Auto ack is enabled for the pipe the packet was received on
    pub fn set_auto_ack(
        &mut self,
        bools: &[bool; PIPES_COUNT],
    ) -> Result<(), Error<SPIE>> {
        // Convert back
        let register = EnAa::from_bools(bools);
        // Write back
        self.write_register(register)?;
        Ok(())
    }

    /// Get address width configuration
    pub fn get_address_width(
        &mut self,
    ) -> Result<u8, Error<SPIE>> {
        let (_, register) = self.read_register::<SetupAw>()?;
        Ok(2 + register.aw())
    }

    /// Set address width configuration
    pub fn set_address_width(&mut self, width: u8)
        -> Result<(), Error<SPIE>> {

        let register = SetupAw(width - 2);
        self.write_register(register)?;
        Ok(())
    }

    /// Obtain interrupt pending status as `(RX_DR, TX_DR, MAX_RT)`
    /// where `RX_DR` indicates new data in the RX FIFO, `TX_DR`
    /// indicates that a packet has been sent, and `MAX_RT` indicates
    /// maximum retransmissions without auto-ack.
    pub fn get_interrupts(
        &mut self,
    ) -> Result<(bool, bool, bool), Error<SPIE>> {
        let (status, ()) = self.send_command(&Nop)?;
        Ok((status.rx_dr(), status.tx_ds(), status.max_rt()))
    }

    /// Clear all interrupts
    pub fn clear_interrupts(
        &mut self,
    ) -> Result<(), Error<SPIE>> {
        let mut clear = Status(0);
        clear.set_rx_dr(true);
        clear.set_tx_ds(true);
        clear.set_max_rt(true);
        self.write_register(clear)?;
        Ok(())
    }

    /// ## `bools`
    /// * `None`: Dynamic payload length
    /// * `Some(len)`: Static payload length `len`
    pub fn set_pipes_rx_lengths(
        &mut self,
        lengths: &[Option<u8>; PIPES_COUNT],
    ) -> Result<(), Error<SPIE>> {
        // Enable dynamic payload lengths
        let mut bools = [true; PIPES_COUNT];
        for (i, length) in lengths.iter().enumerate() {
            bools[i] = length.is_none();
        }
        let dynpd = Dynpd::from_bools(&bools);
        if dynpd.0 != 0 {
            self.update_register::<Feature, _, _>(|feature| {
                feature.set_en_dpl(true);
            })?;
        }
        self.write_register(dynpd)?;

        // Set static payload lengths
        macro_rules! set_rx_pw {
            ($name: ident, $index: expr) => {{
                use crate::registers::$name;
                let length = lengths[$index].unwrap_or(0);
                let mut register = $name(0);
                register.set(length);
                self.write_register(register)?;
            }};
        }
        set_rx_pw!(RxPwP0, 0);
        set_rx_pw!(RxPwP1, 1);
        set_rx_pw!(RxPwP2, 2);
        set_rx_pw!(RxPwP3, 3);
        set_rx_pw!(RxPwP4, 4);
        set_rx_pw!(RxPwP5, 5);

        Ok(())
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
