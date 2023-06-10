#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::fmt::Write;

use cec_adapter::{cec_types, send_with_result, CecDecodeError, CecFrame, LogicalAddress};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::gpio::Pin;
use embassy_rp::gpio::{self, OutputOpenDrain};
use embassy_rp::i2c::{Config, I2c};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};

use gpio::Level;
use heapless::{String, Vec};
use ssd1306::mode::DisplayConfig;
use ssd1306::prelude::Brightness;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::DisplaySize128x64;
use ssd1306::{I2CDisplayInterface, Ssd1306};
use {defmt_rtt as _, panic_probe as _};

use core::write;

use cec_adapter::cec_types::{CecDeviceType, CecOpCode};

#[embassy_executor::task]
async fn alive_logger() {
    loop {
        Timer::after(Duration::from_millis(5000)).await;
        info!("Alive {}!", Instant::now());
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

#[embassy_executor::task(pool_size = 4)]
async fn cec_periodic_message_task(frame: CecFrame, interval: Duration) {
    loop {
        info!("Periodic msg to {}", frame.dest);
        let _ = send_with_result(frame.clone()).await;
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

    let cec0 = OutputOpenDrain::new(p.PIN_0.degrade(), Level::High);
    let my_cec_address = LogicalAddress(5);

    cec_adapter::spawn_cec_handling_task(spawner, cec0, my_cec_address).unwrap();

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

    let _ = send_with_result(CecFrame {
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
    let mut cec_incoming_subscriber = cec_adapter::subscribe_incoming().unwrap();
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
                    write!(int_str, "{}", v).unwrap();
                    operands.push_str(int_str.as_str()).unwrap();
                    operands.push_str(",").unwrap();
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
                            let _ = send_with_result(CecFrame {
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
