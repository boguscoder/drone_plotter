use core::sync::atomic::Ordering;
use crossbeam_channel::{Receiver, Sender};
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

use strum_macros::{AsRefStr, EnumIter};

#[derive(Debug, EnumIter, AsRefStr, PartialEq, Clone, Copy)]
pub enum TeleCategory {
    None = 0,
    Imu,
    Rc,
    Attitude,
    Pid,
    Mix,
    Dshot,
}

use crate::SensorData;
use crate::VALS_PER_LINE;

const IN_PORT_PATH: &str = "/dev/cu.usbmodem0xBABECAFE1";
const OUT_PORT_PATH: &str = "/dev/cu.usbmodem0xBABECAFE3";

const BAUD_RATE: u32 = 115200;
const TELE_MAX_VALUES: u8 = 9;
const TELE_FRAME_SIZE: usize = 2 + (TELE_MAX_VALUES as usize * 4);

fn open_serial_port(port_path: &str) -> serialport::Result<Box<dyn SerialPort>> {
    serialport::new(port_path, BAUD_RATE)
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(10))
        .open()
}

pub fn start_input_threads<F>(
    repaint_fn: F,
    data_channel: Sender<SensorData>,
    msg_channel: Sender<String>,
    cmd_channel: Receiver<TeleCategory>,
) where
    F: Fn() + Send + Sync + 'static,
{
    // Spawn a thread for text logs from the log port
    std::thread::spawn(move || {
        loop {
            if let Ok(port) = open_serial_port(IN_PORT_PATH) {
                let mut reader = BufReader::new(port);
                let mut line = String::new();
                loop {
                    match reader.read_line(&mut line) {
                        Ok(0) => break, // EOF
                        Ok(_) => {
                            let trimmed = line.trim();
                            if !trimmed.is_empty() {
                                let _ = msg_channel.send(trimmed.to_string());
                            }
                            line.clear();
                        }
                        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                            // Timeout is expected, just keep reading
                        }
                        Err(_) => break, // Real error, reconnect
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    });

    // Handle binary telemetry from the app port in the main input thread
    std::thread::spawn(move || {
        let mut last_mode = TeleCategory::None;
        loop {
            if let Ok(mut port) = open_serial_port(OUT_PORT_PATH) {
                // Send last known mode immediately on reconnect
                port.write_all(&[last_mode as u8]).unwrap();

                let mut buf = [0u8; 1024];
                let mut state = 0; // 0: Idle, 1: NeedLen, 2: Collecting
                let mut frame = [0u8; TELE_FRAME_SIZE];
                let mut pos = 0;

                loop {
                    while let Ok(mode) = cmd_channel.try_recv() {
                        last_mode = mode;
                        if port.write_all(&[last_mode as u8]).is_err() {
                            break;
                        }
                    }

                    match port.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => {
                            let val_num = VALS_PER_LINE.load(Ordering::Acquire);
                            for &b in &buf[..n] {
                                match state {
                                    0 => {
                                        if b == 0xAA {
                                            frame[0] = b;
                                            state = 1;
                                        }
                                    }
                                    1 => {
                                        if b > 0 && b <= TELE_MAX_VALUES {
                                            frame[1] = b;
                                            pos = 2;
                                            state = 2;
                                        } else {
                                            state = 0;
                                        }
                                    }
                                    2 => {
                                        frame[pos] = b;
                                        pos += 1;
                                        if pos >= 2 + (frame[1] as usize) * 4 {
                                            let mut values = Vec::new();
                                            for i in 0..(frame[1] as usize) {
                                                let offset = 2 + i * 4;
                                                values.push(f32::from_le_bytes([
                                                    frame[offset],
                                                    frame[offset + 1],
                                                    frame[offset + 2],
                                                    frame[offset + 3],
                                                ])
                                                    as f64);
                                            }
                                            if values.len() == val_num {
                                                let _ =
                                                    data_channel.try_send(SensorData { values });
                                                repaint_fn();
                                            }
                                            state = 0;
                                        }
                                    }
                                    _ => state = 0,
                                }
                            }
                        }
                    }
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    });
}
