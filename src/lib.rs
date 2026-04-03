#![no_std]

mod cec;
pub mod cec_types;

pub use cec::CecDecodeError;
pub use cec::CecFrame;
pub use cec::CecSendError;
pub use cec::LogicalAddress;

pub use cec::receive;
pub use cec::send_with_result;
pub use cec::subscribe_incoming;

pub fn spawn_cec_handling_task(
    spawner: embassy_executor::Spawner,
    pin: embassy_rp::gpio::OutputOpenDrain<'static>,
    dev_addr: LogicalAddress,
) {
    spawner.spawn(defmt::expect!(cec::cec_line_handler(pin, dev_addr)))
}
