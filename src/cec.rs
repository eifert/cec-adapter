use core::iter::once;

use defmt::*;
use embassy_futures::join::join;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_rp::gpio::OutputOpenDrain;
use embassy_rp::gpio::{AnyPin, Pin};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{with_timeout, Duration, Instant, Timer};

use heapless::{Deque, Vec};
use num_enum::IntoPrimitive;

const MAX_CEC_OPERANDS: usize = 16;

const DATA_BIT_NOMINAL_SAMPLE_TIME: Duration = Duration::from_micros(1050);
const DATA_ACK_ASSERTION_PERIOD: Duration = Duration::from_micros(1500);
const DATA_NOMINAL_PERIOD: Duration = Duration::from_micros(2400);

#[derive(Default, Clone, Format)]
pub struct CecFrame {
    pub initiator: LogicalAddress,
    pub dest: LogicalAddress,
    pub opcode: Option<u8>,
    pub operands: Option<heapless::Vec<u8, MAX_CEC_OPERANDS>>,
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
pub struct LogicalAddress(pub u8);

impl LogicalAddress {
    pub fn is_broadcast(&self) -> bool {
        self.0 == 15
    }

    pub fn broadcast() -> LogicalAddress {
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

async fn _await_signal_free<P: Pin>(pin: &mut OutputOpenDrain<'_, P>, kind: SignalFreeKind) {
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
pub enum CecDecodeError {
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
pub enum CecSendError {
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
pub async fn cec_line_handler(mut pin: OutputOpenDrain<'static, AnyPin>, dev_addr: LogicalAddress) {
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
                unwrap!(to_send.push_back(frame));
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
                    (Err(_cec_send_error), retries) if retries.map_or(true, |v| v > 0) => {
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

static SEND_MUTEX: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

pub async fn send_with_result(frame: CecFrame) -> Result<(), CecSendError> {
    let _guard = SEND_MUTEX.lock().await;
    info!("Sending");
    join(
        CEC_OUTGOING_CHANNEL.send(frame),
        CEC_SENDRESULT_CHANNEL.recv(),
    )
    .await
    .1
}

pub async fn receive() -> Result<CecFrame, CecDecodeError> {
    loop {
        match CEC_INCOMING_CHANNEL
            .subscriber()
            .unwrap()
            .next_message()
            .await
        {
            embassy_sync::pubsub::WaitResult::Lagged(_) => continue,
            embassy_sync::pubsub::WaitResult::Message(m) => return m,
        }
    }
}

pub fn subscribe_incoming<'a>() -> Result<
    embassy_sync::pubsub::Subscriber<
        'a,
        CriticalSectionRawMutex,
        Result<CecFrame, CecDecodeError>,
        1,
        5,
        1,
    >,
    embassy_sync::pubsub::Error,
> {
    CEC_INCOMING_CHANNEL.subscriber()
}
