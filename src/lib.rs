#![no_std]
#![feature(type_alias_impl_trait)]

mod cec;
pub mod cec_types;

pub use cec::CecDecodeError;
pub use cec::CecFrame;
pub use cec::CecSendError;
pub use cec::LogicalAddress;

pub use cec::receive;
pub use cec::send_with_result;

pub fn spawn_cec_handling_task(
    spawner: embassy_executor::Spawner,
    pin: embassy_rp::gpio::OutputOpenDrain<'static, embassy_rp::gpio::AnyPin>,
    dev_addr: LogicalAddress,
) -> Result<(), embassy_executor::SpawnError> {
    spawner.spawn(cec::cec_line_handler(pin, dev_addr))
}

pub use cec::subscribe_incoming;
