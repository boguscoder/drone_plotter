use core::sync::atomic::Ordering;
use crossbeam_channel::Sender;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{BufRead, BufReader};
use std::time::Duration;

use crate::SensorData;
use crate::VALS_PER_LINE;

const IN_PORT_PATH: &str = "/dev/tty.usbmodem0xBABECAFE1";
const OUT_PORT_PATH: &str = "/dev/tty.usbmodem0xBABECAFE3";

const BAUD_RATE: u32 = 115200;

fn open_serial_port(port_path: &str) -> serialport::Result<Box<dyn SerialPort>> {
    serialport::new(port_path, BAUD_RATE)
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(10))
        .open()
}

pub fn open_out_port() -> serialport::Result<Box<dyn SerialPort>> {
    open_serial_port(OUT_PORT_PATH)
}

pub fn input_thread(data_channel: Sender<SensorData>, msg_channel: Sender<String>) {
    // Spawn a thread for text logs from the log port
    let msg_clone = msg_channel.clone();
    std::thread::spawn(move || {
        loop {
            if let Ok(port) = open_serial_port(IN_PORT_PATH) {
                let mut reader = BufReader::new(port);
                let mut line = String::new();
                while reader.read_line(&mut line).is_ok() {
                    let trimmed = line.trim();
                    if !trimmed.is_empty() {
                        let _ = msg_clone.send(trimmed.to_string());
                    }
                    line.clear();
                }
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    });

    // Handle binary telemetry from the app port in the main input thread
    loop {
        if let Ok(mut port) = open_serial_port(OUT_PORT_PATH) {
            let mut buf = [0u8; 1024];
            let mut state = 0; // 0: Idle, 1: NeedLen, 2: Collecting
            let mut frame = [0u8; 26];
            let mut pos = 0;

            loop {
                match port.as_mut().read(&mut buf) {
                    Ok(0) | Err(_) => break,
                    Ok(n) => {
                        let val_num = VALS_PER_LINE.load(Ordering::Acquire);
                        for &b in &buf[..n] {
                            match state {
                                0 => if b == 0xAA { frame[0] = b; state = 1; },
                                1 => {
                                    if b > 0 && b <= 6 {
                                        frame[1] = b;
                                        pos = 2;
                                        state = 2;
                                    } else { state = 0; }
                                },
                                2 => {
                                    frame[pos] = b;
                                    pos += 1;
                                    if pos >= 2 + (frame[1] as usize) * 4 {
                                        let mut values = Vec::new();
                                        for i in 0..(frame[1] as usize) {
                                            let offset = 2 + i * 4;
                                            values.push(f32::from_le_bytes([frame[offset], frame[offset+1], frame[offset+2], frame[offset+3]]) as f64);
                                        }
                                        if values.len() == val_num {
                                            let _ = data_channel.try_send(SensorData { values });
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
}
