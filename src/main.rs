#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use core::iter::once;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::gpio::Pin;
use embassy_rp::gpio::{self, OutputOpenDrain};
use embassy_rp::i2c::{Config, I2c};
use embassy_time::{Duration, Instant, Timer};

use gpio::Level;
use heapless::{String, Vec};
use num_enum::IntoPrimitive;
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::Brightness;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

use core::write;

use crate::cec_types::CecOpCode;

#[embassy_executor::task]
async fn alive_logger() {
    loop {
        Timer::after(Duration::from_millis(5000)).await;
        info!("Alive {}!", Instant::now());
    }
}

mod cec_types;

const MAX_CEC_OPERANDS: usize = 64;

const DATA_BIT_NOMINAL_SAMPLE_TIME: Duration = Duration::from_micros(1050);
const DATA_ACK_ASSERTION_PERIOD: Duration = Duration::from_micros(1500);
const DATA_NOMINAL_PERIOD: Duration = Duration::from_micros(2400);

#[derive(Default, Clone)]
struct CecFrame {
    initiator: LogicalAddress,
    dest: LogicalAddress,
    opcode: Option<u8>,
    operands: Option<heapless::Vec<u8, MAX_CEC_OPERANDS>>,
}

impl CecFrame {
    fn is_polling_message(&self) -> bool {
        self.initiator == self.dest && self.opcode.is_none() && self.operands.is_none()
    }
}

#[derive(Default, Copy, Clone, PartialEq, Eq, defmt::Format)]
struct LogicalAddress(u8);

impl LogicalAddress {
    fn is_broadcast(&self) -> bool {
        self.0 == 15
    }
}

#[repr(u8)]
#[derive(IntoPrimitive, Clone, Copy, PartialEq, Eq)]
enum SignalFreeKind {
    SameInitiator = 5,
    NewInitiator = 7,
    Retransmit = 3,
}

async fn await_signal_free<P: Pin>(pin: &mut OutputOpenDrain<'_, P>, kind: SignalFreeKind) {
    // Cf. CEC 9.1: Signal Free Time (HDMI spec 1.4)
    if pin.is_low() {
        pin.wait_for_rising_edge().await;
    }
    loop {
        if matches!(
            select(
                pin.wait_for_falling_edge(),
                Timer::after(u8::from(kind) as u32 * DATA_NOMINAL_PERIOD),
            )
            .await,
            Either::Second(_)
        ) {
            break;
        }
    }
}

enum CecDecodeError {
    Nack(CecFrame),
    Other(&'static str),
}

async fn cec_decode<P: Pin>(
    pin: &mut OutputOpenDrain<'_, P>,
    logical_address: LogicalAddress,
) -> Result<CecFrame, CecDecodeError> {
    await_signal_free(pin, SignalFreeKind::SameInitiator).await;

    {
        // Start bit

        pin.wait_for_falling_edge().await;
        let frame_start = Instant::now();
        pin.wait_for_rising_edge().await;
        let low_duration = Instant::now() - frame_start;
        pin.wait_for_falling_edge().await;
        let total_duration = Instant::now() - frame_start;
        if !(low_duration >= Duration::from_micros(3500)
            && low_duration <= Duration::from_micros(3900)
            && total_duration >= Duration::from_micros(4300)
            && total_duration <= Duration::from_micros(4700))
        {
            return Err(CecDecodeError::Other("Bad start bit"));
        }
    }
    let mut data = Vec::<u8, MAX_CEC_OPERANDS>::new();
    let mut have_nack_blocks = false;
    'block: loop {
        //info!("cec_decode block, have {}", data.len());
        let mut bits = 0u8;
        let mut eom = false;
        for bit in 0..=8 {
            Timer::after(DATA_BIT_NOMINAL_SAMPLE_TIME).await;

            match bit {
                0..=7 => {
                    if pin.is_high() {
                        bits |= 1 << (7 - bit)
                    }
                }
                8 => eom = pin.is_high(),

                _ => {}
            }

            pin.wait_for_falling_edge().await;
        }

        let dest = LogicalAddress(if data.is_empty() {
            bits & 0x0f
        } else {
            data[0] & 0x0f
        });

        let mut ack_low = false;
        {
            //ACK handling
            if dest == logical_address {
                info!("ACK for {} ", dest.0);
                pin.set_low();
                Timer::after(DATA_ACK_ASSERTION_PERIOD).await;
                pin.set_high();
            } else {
                Timer::after(DATA_BIT_NOMINAL_SAMPLE_TIME).await;
                if pin.is_low() {
                    // ack has inverted logic
                    ack_low = true;
                }
            }
        }

        data.push(bits).unwrap();
        if (!dest.is_broadcast() && ack_low) || (dest.is_broadcast() && !ack_low) {
        } else {
            have_nack_blocks = true;
        }

        if eom {
            break 'block;
        }
        pin.wait_for_falling_edge().await;
    }
    if data.is_empty() {
        return Err(CecDecodeError::Other("No blocks"));
    }

    let frame = CecFrame {
        initiator: LogicalAddress((data[0] & 0xf0) >> 4),
        dest: LogicalAddress(data[0] & 0x0f),
        opcode: data.get(1).copied(),
        operands: data.get(2..).map(|v| Vec::from_slice(v).unwrap()),
    };
    if have_nack_blocks {
        Err(CecDecodeError::Nack(frame))
    } else {
        Ok(frame)
    }
}

#[derive(defmt::Format)]
enum CecSendError {
    Nack,
    _Other(&'static str),
}

async fn cec_send<P: Pin>(
    pin: &mut OutputOpenDrain<'_, P>,
    frame: &CecFrame,
) -> Result<(), CecSendError> {
    //Cf. CEC 7.1 Frame Re-transmissions
    let max_attempts = if frame.is_polling_message() { 2 } else { 5 };
    for frame_attempt in 0..max_attempts {
        let mut have_nack_blocks = false;
        await_signal_free(
            pin,
            if frame_attempt == 0 {
                SignalFreeKind::NewInitiator
            } else {
                SignalFreeKind::Retransmit
            },
        )
        .await;
        {
            //Start bit
            pin.set_low();
            Timer::after(Duration::from_micros(3700)).await;
            pin.set_high();
            Timer::after(Duration::from_micros(800)).await;
        }
        let total_blocks =
            1 + frame.opcode.iter().count() + frame.operands.as_ref().map_or(0, |v| v.len());
        for (idx, payload) in once((frame.initiator.0 << 4) | frame.dest.0)
            .chain(
                frame
                    .opcode
                    .iter()
                    .copied()
                    .chain(frame.operands.iter().flatten().copied()),
            )
            .enumerate()
        {
            let eom = idx == total_blocks - 1;
            info!("Sending block {:x}: {:x} is EOM: {}", idx, payload, eom);
            for bit_idx in 0..7 {
                let bit = ((payload >> (7 - bit_idx)) & 1) == 1;
                pin.set_low();
                Timer::after(Duration::from_micros(if bit { 600 } else { 1500 })).await;
                pin.set_high();
                Timer::after(Duration::from_micros(if bit { 1800 } else { 900 })).await;
            }
            {
                //EOM
                pin.set_low();
                Timer::after(Duration::from_micros(if eom { 600 } else { 1500 })).await;
                pin.set_high();
                Timer::after(Duration::from_micros(if eom { 1800 } else { 900 })).await;
            }
            {
                //ACK
                pin.set_low();
                Timer::after(Duration::from_micros(600)).await;
                pin.set_high();
                Timer::after(Duration::from_micros(450)).await;
                let ack_low = pin.is_low();
                Timer::after(Duration::from_micros(1350)).await;
                if (frame.dest.is_broadcast() && ack_low)
                    || (!frame.dest.is_broadcast() && !ack_low)
                {
                    have_nack_blocks = true;
                }
            };
        }
        if !have_nack_blocks {
            return Ok(());
        }
    }
    Err(CecSendError::Nack)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    //spawner.spawn(alive_logger()).unwrap();

    let mut cec0 = OutputOpenDrain::new(p.PIN_0, Level::High);

    info!("set up i2c ");
    let mut config = Config::default();
    config.frequency = 1_000_000;

    let scl = p.PIN_15;
    let sda = p.PIN_14;
    let i2c = I2c::new_blocking(p.I2C1, scl, sda, config);

    let interface = I2CDisplayInterface::new(i2c);
    let mut terminal =
        Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0).into_terminal_mode();
    terminal.init().unwrap();
    terminal.clear().unwrap();
    terminal.set_brightness(Brightness::NORMAL).unwrap();
    terminal.set_display_on(true).unwrap();
    terminal.write_str("Hi, I'm CEC-Igel").unwrap();

    let my_cec_address = LogicalAddress(14);

    if cec_send(
        &mut cec0,
        &CecFrame {
            initiator: my_cec_address,
            dest: my_cec_address,
            opcode: None,
            operands: None,
        },
    )
    .await
    .is_ok()
    {
        defmt::panic!("Logical Addr for display already allocated");
    }

    info!("Listening for messages");
    loop {
        let frame = cec_decode(&mut cec0, my_cec_address).await;
        if terminal.position().unwrap().1 == 0 {
            terminal.clear().unwrap();
        }

        match frame {
            Err(CecDecodeError::Nack(CecFrame {
                initiator: LogicalAddress(4),
                dest: LogicalAddress(4),
                opcode: None,
                operands: None,
            })) => {
                info!("Got addr 4 allocation, querying for phys addr");

                let res = cec_send(
                    &mut cec0,
                    &CecFrame {
                        initiator: my_cec_address,
                        dest: LogicalAddress(4),
                        opcode: Some(CecOpCode::GET_CEC_VERSION as u8),
                        operands: None,
                    },
                )
                .await;
                if res.is_err() {
                    error!("No ack when querying log dev 4");
                }
            }
            Err(CecDecodeError::Nack(nack_frame)) => {
                info!(
                    "Nack frame {}->{}",
                    nack_frame.initiator.0, nack_frame.dest.0
                );
            }
            Ok(frame) => {
                const OP_STR_LEN: usize = 32usize;
                let op_str: String<OP_STR_LEN> = frame
                    .opcode
                    .map(|opcode| {
                        cec_types::CecOpCode::try_from(opcode)
                            .map(|v| {
                                let s: &str = v.into();
                                String::<OP_STR_LEN>::from(s)
                            })
                            .unwrap_or(String::<OP_STR_LEN>::from(opcode))
                    })
                    .unwrap_or(String::<OP_STR_LEN>::from("(None)"));
                info!(
                    "{} -> {} {} {}",
                    frame.initiator,
                    frame.dest,
                    op_str.as_str(),
                    frame.operands.map(|v| v.len())
                );
                write!(
                    terminal,
                    "{}->{} {}\n",
                    frame.initiator.0,
                    frame.dest.0,
                    &op_str.as_str().get(0..9).unwrap_or(op_str.as_str())
                )
                .unwrap();
            }
            Err(CecDecodeError::Other(msg)) => {
                error!("Bad frame {}", msg);
                write!(terminal, "{}\n", msg).unwrap();
            }
        }
    }
}
