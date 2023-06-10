#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;
use core::iter::once;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_rp::gpio::{self, OutputOpenDrain};
use embassy_rp::gpio::{AnyPin, Pin};
use embassy_rp::i2c::{Config, I2c};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::{PubSubChannel, WaitResult};
use embassy_time::{with_timeout, Duration, Instant, Timer};

use gpio::Level;
use heapless::{Deque, String, Vec};
use num_enum::IntoPrimitive;
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::Brightness;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

use core::write;

use crate::cec_types::{CecDeviceType, CecOpCode};

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

    fn recommended_send_retries(&self) -> usize {
        if self.is_polling_message() {
            2
        } else {
            5
        }
    }
}

#[derive(Default, Copy, Clone, PartialEq, Eq, defmt::Format)]
struct LogicalAddress(u8);

impl LogicalAddress {
    fn is_broadcast(&self) -> bool {
        self.0 == 15
    }

    fn broadcast() -> LogicalAddress {
        LogicalAddress(15)
    }
}

#[repr(u8)]
#[derive(IntoPrimitive, Clone, Copy, PartialEq, Eq)]
enum SignalFreeKind {
    SameInitiator = 5,
    NewInitiator = 7,
    Retransmit = 3,
}

impl SignalFreeKind {
    fn required_free_duration(&self) -> Duration {
        u8::from(*self) as u32 * DATA_NOMINAL_PERIOD
    }
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
                Timer::after(kind.required_free_duration()),
            )
            .await,
            Either::Second(_)
        ) {
            break;
        }
    }
}

#[derive(Clone)]
enum CecDecodeError {
    Nack(CecFrame),
    Other(&'static str),
}

async fn cec_decode<P: Pin>(
    pin: &mut OutputOpenDrain<'_, P>,
    logical_address: LogicalAddress,
) -> Result<CecFrame, CecDecodeError> {
    let mut data = Vec::<u8, MAX_CEC_OPERANDS>::new();
    let mut have_nack_blocks = false;
    let mut eom_fused = false;
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

        let ack_block_res = with_timeout(Duration::from_micros(2750), async {
            let mut ack_low = false;
            {
                //ACK handling
                if dest == logical_address {
                    info!("ACK for {} ", dest.0);
                    pin.set_low();
                    Timer::after(DATA_ACK_ASSERTION_PERIOD).await;
                    pin.set_high();
                    ack_low = true;
                } else {
                    Timer::after(DATA_BIT_NOMINAL_SAMPLE_TIME).await;
                    if pin.is_low() {
                        // ack has inverted logic
                        ack_low = true;
                    }
                }
            }
            if !eom_fused {
                data.push(bits).unwrap();
            }
            eom_fused = eom_fused || eom;
            if (!dest.is_broadcast() && ack_low) || (dest.is_broadcast() && !ack_low) {
            } else {
                have_nack_blocks = true;
            }
            pin.wait_for_falling_edge().await;
        })
        .await;

        if ack_block_res.is_err() {
            // No more falling edges/blocks after eom, we can consider the frame done
            break 'block;
        }
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
    let mut have_nack_blocks = false;
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
        for bit_idx in 0..=7 {
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
            if (frame.dest.is_broadcast() && ack_low) || (!frame.dest.is_broadcast() && !ack_low) {
                have_nack_blocks = true;
            }
        }
    }
    if !have_nack_blocks {
        Ok(())
    } else {
        Err(CecSendError::Nack)
    }
}

static CEC_OUTGOING_CHANNEL: Channel<CriticalSectionRawMutex, CecFrame, 1> = Channel::new();
static CEC_SENDRESULT_CHANNEL: Channel<CriticalSectionRawMutex, Result<(), CecSendError>, 1> =
    Channel::new();
static CEC_INCOMING_CHANNEL: PubSubChannel<
    CriticalSectionRawMutex,
    Result<CecFrame, CecDecodeError>,
    1,
    5,
    1,
> = PubSubChannel::new();

#[embassy_executor::task]
async fn cec_line_handler(mut pin: OutputOpenDrain<'static, AnyPin>, dev_addr: LogicalAddress) {
    let mut to_send = Deque::<CecFrame, 8>::new();
    let mut send_wait: Option<Duration> = None;
    let mut retry_frame = None;
    let mut retries_remaining = None;
    pin.wait_for_high().await;
    let mut free_since = Instant::now();
    let incoming_frame_publisher = CEC_INCOMING_CHANNEL.publisher().unwrap();
    loop {
        if pin.is_low() {
            pin.wait_for_high().await;
            free_since = Instant::now();
        }
        //info!("Waiting for send {}", remaining_wait);
        let free_for = Instant::now() - free_since;
        let remaining_wait = send_wait.map_or(Duration::from_secs(999999), |send_wait| {
            send_wait
                .checked_sub(free_for)
                .unwrap_or(Duration::from_micros(0))
        });

        match select3(
            pin.wait_for_falling_edge(),
            CEC_OUTGOING_CHANNEL.recv(),
            Timer::after(remaining_wait),
        )
        .await
        {
            Either3::First(_) => {
                // Check for valid start bit

                match with_timeout(Duration::from_micros(5000), async {
                    let frame_start = Instant::now();
                    pin.wait_for_rising_edge().await;
                    let low_duration = Instant::now() - frame_start;
                    pin.wait_for_falling_edge().await;
                    let total_duration = Instant::now() - frame_start;
                    let good_start_bit = low_duration >= Duration::from_micros(3500)
                        && low_duration <= Duration::from_micros(3900)
                        && total_duration >= Duration::from_micros(4300)
                        && total_duration <= Duration::from_micros(4700);
                    if !good_start_bit {
                        info!("Start bit low {} total {}", low_duration, total_duration);
                    }
                    good_start_bit
                })
                .await
                {
                    Ok(true) => {
                        let _ = with_timeout(Duration::from_millis(200), async {
                            let frame = cec_decode(&mut pin, dev_addr).await;
                            incoming_frame_publisher.publish(frame).await;
                        })
                        .await;

                        pin.set_high(); // In case we cancelled during cec_decode acknowledge, force to well-defined high
                    }

                    Ok(false) => {
                        incoming_frame_publisher
                            .publish(Err(CecDecodeError::Other("Start bit timing out of range")))
                            .await;
                    }
                    _ => {
                        incoming_frame_publisher
                            .publish(Err(CecDecodeError::Other("Start bit detection timed out")))
                            .await;
                    }
                }

                pin.wait_for_high().await;
                free_since = Instant::now();
            }
            Either3::Second(frame) => {
                to_send.push_back(frame);
                if send_wait.is_none() {
                    send_wait = Some(
                        SignalFreeKind::NewInitiator
                            .required_free_duration()
                            .checked_sub(Instant::now() - free_since)
                            .unwrap_or(Duration::from_micros(0)),
                    );
                }

                // TODO reduce remaining wait according to elapsed wait until send request came in
                /* info!(
                    "Queued send message at {}, waiting {}",
                    Instant::now(),
                    remaining_wait
                );*/
            }
            Either3::Third(_) => {
                send_wait = None;

                let frame = match retry_frame {
                    Some(frame) => {
                        //info!("Retry message at {}", Instant::now());
                        frame
                    }
                    None => {
                        //info!("Unqueued send message at {}", Instant::now());
                        to_send.pop_front().unwrap()
                    }
                };

                match (cec_send(&mut pin, &frame).await, retries_remaining) {
                    (Err(cec_send_error), retries) if retries.map_or(true, |v| v > 0) => {
                        //Cf. CEC 7.1 Frame Re-transmissions
                        retries_remaining = retries_remaining
                            .map_or(Some(frame.recommended_send_retries() - 1), |v: usize| {
                                Some(v.saturating_sub(1))
                            });
                        /* info!(
                            "Send failed with {}, retries {}",
                            cec_send_error, retries_remaining
                        ); */
                        retry_frame = Some(frame);
                        send_wait = Some(SignalFreeKind::Retransmit.required_free_duration());
                    }
                    (res, _) => {
                        if retries_remaining.is_some() {
                            info!(
                                "Finally giving up msg retries, {} other msgs remaining",
                                to_send.len()
                            );
                        }
                        retries_remaining = None;
                        retry_frame = None;
                        CEC_SENDRESULT_CHANNEL.send(res).await;
                        if !to_send.is_empty() {
                            send_wait =
                                Some(SignalFreeKind::SameInitiator.required_free_duration());
                        }
                    }
                }
                pin.wait_for_high().await;
                free_since = Instant::now();
            }
        }
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn cec_device_alive_poller(
    dev_address: LogicalAddress,
    remote_addr: LogicalAddress,

    interval: Duration,
) {
    loop {
        match send_with_result({
            CecFrame {
                initiator: dev_address,
                dest: remote_addr,
                opcode: None,
                operands: None,
            }
        })
        .await
        {
            Ok(_) => info!("Polling {}: Alive!", remote_addr),
            Err(_) => info!("Polling {}: No answer", remote_addr),
        }
        Timer::after(interval).await;
    }
}

static SEND_MUTEX: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

async fn send_with_result(frame: CecFrame) -> Result<(), CecSendError> {
    let _guard = SEND_MUTEX.lock().await;
    info!("Sending");
    join(
        CEC_OUTGOING_CHANNEL.send(frame),
        CEC_SENDRESULT_CHANNEL.recv(),
    )
    .await
    .1
}

#[embassy_executor::task(pool_size = 4)]
async fn cec_periodic_message_task(frame: CecFrame, interval: Duration) {
    loop {
        info!("Periodic msg to {}", frame.dest);
        send_with_result(frame.clone()).await;
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

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

    let mut cec0 = OutputOpenDrain::new(p.PIN_0.degrade(), Level::High);
    let my_cec_address = LogicalAddress(5);
    spawner
        .spawn(cec_line_handler(cec0, my_cec_address))
        .unwrap();

    if (send_with_result(CecFrame {
        initiator: my_cec_address,
        dest: my_cec_address,
        opcode: None,
        operands: None,
    })
    .await)
        .is_ok()
    {
        defmt::panic!("Logical Addr for display already allocated");
    }

    send_with_result(CecFrame {
        initiator: my_cec_address,
        dest: LogicalAddress::broadcast(),
        opcode: Some(CecOpCode::REPORT_PHYSICAL_ADDRESS as u8),
        operands: Some(
            Vec::from_slice(&[0x12, 0x34, CecDeviceType::PLAYBACK_DEVICE as u8]).unwrap(),
        ),
    })
    .await;

    spawner
        .spawn(cec_device_alive_poller(
            my_cec_address,
            LogicalAddress(4),
            Duration::from_secs(5),
        ))
        .unwrap();

    spawner
        .spawn(cec_device_alive_poller(
            my_cec_address,
            LogicalAddress(0),
            Duration::from_secs(5),
        ))
        .unwrap();

    spawner
        .spawn(cec_periodic_message_task(
            CecFrame {
                initiator: my_cec_address,
                dest: LogicalAddress(4),
                opcode: Some(CecOpCode::GIVE_PHYSICAL_ADDRESS as u8),
                operands: None,
            },
            Duration::from_secs(5),
        ))
        .unwrap();

    info!("Listening for messages");
    let mut cec_incoming_subscriber = CEC_INCOMING_CHANNEL.subscriber().unwrap();
    loop {
        let frame = match cec_incoming_subscriber.next_message().await {
            WaitResult::Lagged(_) => {
                continue;
            }
            WaitResult::Message(m) => m,
        };
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
                /*info!("Got addr 4 allocation, querying for phys addr");

                let res = send_with_result(CecFrame {
                        initiator: my_cec_address,
                        dest: LogicalAddress(4),
                        opcode: Some(CecOpCode::GET_CEC_VERSION as u8),
                        operands: None,
                    })
                    .await;*/
                //TODO
                /*if res.is_err() {
                    error!("No ack when querying log dev 4");
                }*/
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

                let mut operands = String::<100>::new();
                frame.operands.iter().flatten().for_each(|v| {
                    let mut int_str = String::<30>::new();
                    write!(int_str, "{}", v);
                    operands.push_str(int_str.as_str()).unwrap();
                    operands.push_str(",");
                });
                info!(
                    "{} -> {} {} {}",
                    frame.initiator,
                    frame.dest,
                    op_str.as_str(),
                    operands.as_str(),
                );
                write!(
                    terminal,
                    "{}->{} {}\n",
                    frame.initiator.0,
                    frame.dest.0,
                    &op_str.as_str().get(0..9).unwrap_or(op_str.as_str())
                )
                .unwrap();

                if frame.dest == my_cec_address {
                    if let Some(opcode) = frame.opcode {
                        if opcode == CecOpCode::GIVE_DEVICE_VENDOR_ID as u8 {
                            let res = send_with_result(CecFrame {
                                initiator: my_cec_address,
                                dest: LogicalAddress::broadcast(),
                                opcode: Some(CecOpCode::DEVICE_VENDOR_ID as u8),
                                operands: Some(Vec::from_slice(&[0xAF, 0xFE, 0x42]).unwrap()),
                            })
                            .await;
                        }
                    }
                }
            }
            Err(CecDecodeError::Other(msg)) => {
                error!("Bad frame {}", msg);
                write!(terminal, "{}\n", msg).unwrap();
            }
        }
    }
}
