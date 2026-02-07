//! LS7366 Buffer encoder interface using `embedded_hal`.
//!
//! This driver should work with any SPI interface as long as it implements
//! the blocking `embedded_hal` [`SPI traits`].
//!
//! The library is built with `no_std`.
//!
//!
//! # Examples
//! Bare-minimum boilerplate to read from the buffer:
//! ```no_run
//!   use ls7366::Ls7366;
//! // --- snip ---
//! # use std::error::Error;
//! #
//! # use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
//! #
//! # use std::thread::sleep;
//! # use std::time::Duration;
//! # fn main() {
//! #    // create an instance of an SPI object
//! #    // In this case, the buffer is on SPI0 and SS1.
//! #    // The chip acts in Mode0.
//! #    let some_hal_spi_object = Spi::new(Bus::Spi0, SlaveSelect::Ss1, 14_000_000, Mode::Mode0).unwrap();
//! #    let some_cs_pin = todo!();
//! #
//!     // Construct a driver instance from the SPI interface, using default chip configurations.
//!     let mut spi_driver = Ls7366::new(some_hal_spi_object, some_cs_pin).unwrap();
//!
//!     // Loop and read the counter.
//!     loop {
//!         let result = spi_driver.get_count().unwrap();
//!         sleep(Duration::from_secs(1));
//!         println!("read data:= {:?}", result);
//!     }
//! // --- snip ---
//! # }
//! ```
//! ## Advanced configuration
//! The LS7366 has two registers dedicated to configuring the chip's various functions:
//! [`Mdr0`] and [`Mdr1`].
//!
//! Configuring the chip can be accomplished by writing into these two registers.
//!
//! **Manually configuring these registers is **not** required when using [`Ls7366::new`].**
//!
//! 1. Build an instance of [`Mdr0`] and [`Mdr1`] with the desired configuration.
//! 2. Write these instances into the relevant registers.
//! ```ignore
//! use ls7366::mdr0::{QuadCountMode, CycleCountMode, FilterClockDivisionFactor,IndexMode, Mdr0};
//! use ls7366::mdr1::{CounterMode, Mdr1};
//! use ls7366::{Ls7366, Target, Encodable};
//! // --- snip ---
//!     let mdr0_configuration = Mdr0{
//!         quad_count_mode: QuadCountMode::Quad2x,
//!         filter_clock : FilterClockDivisionFactor::Two,
//!         index_mode: IndexMode::ClearCntr,
//!         cycle_count_mode: CycleCountMode::SingleCycle,
//!         is_index_inverted: false
//!     };
//!     let mdr1_configuration = Mdr1{
//!         counter_mode: CounterMode::Byte3,
//!         disable_counting:true,
//!         flag_on_bw: false,
//!         flag_on_idx: false,
//!         flag_on_cmp: false,
//!         flag_on_cy: false,
//!     };
//!
//!     // driver.write_register(Target::Mdr0, &[mdr0_configuration.encode()]).unwrap();
//!     // driver.write_register(Target::Mdr1, &[mdr1_configuration.encode()]).unwrap();
//!
//! ```
//!
//! [`SPI traits`]: https://docs.rs/embedded-hal/0.2.3/embedded_hal/blocking/spi/index.html
//! [`Mdr0`]: ./mdr0/struct.Mdr0.html
//! [`Mdr1`]: ./mdr1/struct.Mdr1.html
//! [`Ls7366::new`]: ./struct.Ls7366.html#method.new
#![cfg_attr(not(test), no_std)]

use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;

pub use crate::ir::{Action, Target};
use crate::ir::InstructionRegister;
use crate::str_register::Str;
use crate::traits::Decodable;
pub use crate::traits::Encodable;

pub mod mdr0;
pub mod ir;
pub mod mdr1;
pub mod str_register;
mod traits;
mod errors;
mod utilities;
mod test_instruction_register;

#[derive(Clone, Debug)]
pub enum Error<SpiError> {
    // Underlying SPI interface error
    SpiError(SpiError),
    // Failed to encode / decode payload
    EncodeError(errors::EncoderError),
    // Request to write payload larger than target register.
    PayloadTooBig,
    // CS pin error
    CsError,
}


impl<E: core::fmt::Debug> core::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::SpiError(error)
    }
}

/// An LS7366 Quadrature encoder buffer
pub struct Ls7366<SPI, CS> {
    /// SPI interface where the buffer is attached.
    interface: SPI,
    /// Chip select pin (directly managed by the driver).
    cs: CS,
}

impl<SPI, CS, SpiError> Ls7366<SPI, CS>
    where SPI: Transfer<u8, Error=SpiError> + Write<u8, Error=SpiError>,
          CS: OutputPin {
    /// Creates a new driver and initializes the Chip to some sensible default values.
    /// This will zero the chip's counter, configure it to 4 byte count mode (full range)
    /// and to treat every 4th quadrature pulse as a increment.
    ///
    /// If the chip is already configured or another configuration is preferable,
    /// use the ([`uninit`]) constructor.
    ///
    /// [`uninit`]: #method.new_uninit
    pub fn new(iface: SPI, mut cs: CS) -> Result<Self, Error<SpiError>> {
        cs.set_high().map_err(|_| Error::CsError)?;
        let mut driver = Ls7366 {
            interface: iface,
            cs,
        };
        // Creating configurations for the two MDR configuration registers
        let mdr0_payload = mdr0::Mdr0 {
            quad_count_mode: mdr0::QuadCountMode::Quad4x,
            cycle_count_mode: mdr0::CycleCountMode::FreeRunning,
            index_mode: mdr0::IndexMode::DisableIndex,
            is_index_inverted: false,
            filter_clock: mdr0::FilterClockDivisionFactor::One,
        };
        let mdr1_payload = mdr1::Mdr1 {
            counter_mode: mdr1::CounterMode::Byte4,
            disable_counting: false,
            flag_on_idx: false,
            flag_on_cmp: false,
            flag_on_bw: false,
            flag_on_cy: false,
        };

        // Write primary configuration to chip.
        driver.write_register(ir::Target::Mdr0, &[mdr0_payload.encode()])?;
        // Write secondary configuration to chip.
        driver.write_register(ir::Target::Mdr1, &[mdr1_payload.encode()])?;
        // Zero Dtr to prepare a write into Cntr.
        driver.write_register(ir::Target::Dtr, &[0x00, 0x00, 0x00, 0x00])?;
        // Load Dtr into Cntr.
        driver.act(
            ir::InstructionRegister {
                target: ir::Target::Cntr,
                action: ir::Action::Load,
            },
            &mut [0x00],
        )?;
        // clear status register.
        driver.clear_status()?;
        Ok(driver)
    }

    /// Creates a new driver but does NOT do any initialization actions against the chip.
    pub fn new_uninit(iface: SPI, mut cs: CS) -> Result<Self, Error<SpiError>> {
        cs.set_high().map_err(|_| Error::CsError)?;
        Ok(Ls7366 {
            interface: iface,
            cs,
        })
    }

    /// Release the SPI interface and CS pin.
    pub fn free(self) -> (SPI, CS) {
        (self.interface, self.cs)
    }

    /// Writes bytes into the specified register. attempting to write more than 4 bytes is an error.
    pub fn write_register(&mut self, target: ir::Target, data: &[u8]) -> Result<(), Error<SpiError>> {
        let ir_cmd = ir::InstructionRegister {
            target,
            action: ir::Action::Write,
        };
        if data.len() > 4 {
            return Err(Error::PayloadTooBig);
        }

        let encoded = ir_cmd.encode();
        let payload: &mut [u8] = &mut [encoded, encoded, encoded, encoded, encoded];

        let mut i = 1;
        for datum in data {
            payload[i] = *datum;
            i+=1;
        }
        // only write as many bits as we had data, +1 for the IR.
        self.cs.set_low().map_err(|_| Error::CsError)?;
        let result = self.interface.write(&payload[0.. data.len()+1]);
        self.cs.set_high().map_err(|_| Error::CsError)?;
        result?;
        Ok(())
    }
    /// Executes a read operation against specified register, returning up to 4 bytes from the chip.
    ///
    /// ## Note:
    ///
    /// Reading from [`Str`] clears the register to zero.
    ///
    /// Reading from [`Dtr`] is a Noop.
    ///
    /// Reading from [`Cntr`] overwrites [`Otr`].
    ///
    /// [`Str`]:  ir/enum.Target.html#variant.Str
    /// [`Dtr`]:  ir/enum.Target.html#variant.Dtr
    /// [`Cntr`]: ir/enum.Target.html#variant.Cntr
    /// [`Otr`]:  ir/enum.Target.html#variant.Otr
    pub fn read_register<'a>(&mut self, rx_buffer: &'a mut [u8], target: ir::Target) -> Result<&'a [u8], Error<SpiError>> {
        let ir = ir::InstructionRegister {
            target,
            action: Action::Read,
        };
        let tx_buffer = &mut [ir.encode(), 0x00, 0x00, 0x00, 0x00];

        self.cs.set_low().map_err(|_| Error::CsError)?;
        let result = self.interface.transfer(tx_buffer);
        self.cs.set_high().map_err(|_| Error::CsError)?;
        let result = result?;
        rx_buffer.copy_from_slice(&result[1..]);
        Ok(rx_buffer)
    }
    pub fn get_status(&mut self) -> Result<Str, Error<SpiError>> {
        let result: &mut [u8] = &mut [0x00, 0x00, 0x00, 0x00];
        let raw_result = self.read_register(result, ir::Target::Str)?;
        let result = Str::decode(raw_result[3]);
        match result {
            Ok(data) => Ok(data),
            Err(error) => Err(Error::EncodeError(error)),
        }
    }
    /// Clears the [`Str`] status register to zero.
    ///
    /// [`Str`]:  ir/enum.Target.html#variant.Str
    pub fn clear_status(&mut self) -> Result<(), Error<SpiError>> {
        self.act(
            ir::InstructionRegister {
                target: Target::Str,
                action: Action::Clear,
            }, &mut [0x00],
        )?;
        Ok(())
    }
    /// Reads the chip's current count, sets the sign bit appropriate to the status register
    pub fn get_count(&mut self) -> Result<i64, Error<SpiError>> {
        let raw_result: &mut [u8] = &mut [0x00, 0x00, 0x00, 0x00];
        let raw_result = self.read_register(raw_result,ir::Target::Cntr)?;
        let status = self.get_status()?;
        let count = utilities::vec_to_i64(&raw_result);
        match status.sign_bit {
            str_register::SignBit::Negative => Ok(count * -1),
            str_register::SignBit::Positive => Ok(count),
        }
    }


    /// Performs a transaction against the chip.
    ///
    /// Some actions (e.g. writing to a register) accept up to 4 u8 bytes, this function accepts
    /// the same. Attempt to write more than 4 bytes will result in an ['SpiError.PayloadToBig`]
    ///
    /// Other sources of error responses may arise from the underlying HAL implementation and are
    /// bubbled up.
    pub fn act<'a>(&mut self, command: InstructionRegister, data: &'a mut [u8]) -> Result<& 'a [u8], Error<SpiError>> {
        let tx_buffer: &[u8] = &[command.encode()];
        match command.action {
            Action::Clear | Action::Load => {
                if data.len() > 1 {
                    Err(Error::PayloadTooBig)
                } else {
                    self.cs.set_low().map_err(|_| Error::CsError)?;
                    let result = self.interface.write(&tx_buffer);
                    self.cs.set_high().map_err(|_| Error::CsError)?;
                    result?;
                    Ok(data)
                }
            }
            Action::Read => {
                let mut tx_buffer = [tx_buffer[0], 0x00, 0x00, 0x00, 0x00];
                self.cs.set_low().map_err(|_| Error::CsError)?;
                let result = self.interface.transfer(&mut tx_buffer);
                self.cs.set_high().map_err(|_| Error::CsError)?;
                let result = result?;
                data.copy_from_slice(result);
                Ok(data)
            }
            Action::Write => {
                if data.len() > 5 {
                    Err(Error::PayloadTooBig)
                } else {
                    self.cs.set_low().map_err(|_| Error::CsError)?;
                    let result = self.interface.write(&tx_buffer);
                    self.cs.set_high().map_err(|_| Error::CsError)?;
                    result?;
                    Ok(data)
                }
            }
        }
    }
}
