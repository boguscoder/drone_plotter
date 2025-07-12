use core::sync::atomic::Ordering;
use crossbeam_channel::Sender;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::io::{BufRead, BufReader};
use std::time::Duration;

use crate::SensorData;
use crate::VALS_PER_LINE;

const IN_PORT_PATH: &str = "/dev/tty.usbmodem0xC0DECAFE1";
const OUT_PORT_PATH: &str = "/dev/tty.usbmodem0xC0DECAFE3";

const BAUD_RATE: u32 = 115200;

fn open_serial_port(port_path: &str) -> serialport::Result<Box<dyn SerialPort>> {
    serialport::new(port_path, BAUD_RATE)
        .data_bits(DataBits::Eight)
        .flow_control(FlowControl::None)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .timeout(Duration::from_millis(100))
        .open()
}

pub fn open_out_port() -> serialport::Result<Box<dyn SerialPort>> {
    open_serial_port(OUT_PORT_PATH)
}

pub fn input_thread(data_channel: Sender<SensorData>, msg_channel: Sender<String>) {
    let mut log_port = match open_serial_port(IN_PORT_PATH) {
        Ok(port) => port,
        Err(e) => {
            msg_channel
                .send(format!("Failed to open log port {}: {:?}", IN_PORT_PATH, e))
                .unwrap();
            return;
        }
    };

    let reader = BufReader::new(log_port.as_mut());

    for line_result in reader.lines() {
        if let Ok(line) = line_result {
            let values: Vec<f64> = line
                .split(',')
                .filter_map(|s| {
                    s.parse().ok().and_then(|val: f64| {
                        if val.is_finite() {
                            Some(val)
                        } else {
                            msg_channel
                                .send(format!("Skipping non-finite value: {}", s))
                                .unwrap();
                            None
                        }
                    })
                })
                .collect();

            let val_num = VALS_PER_LINE.load(Ordering::Acquire);
            if values.len() == val_num {
                let sensor_data = SensorData { values };
                data_channel.send(sensor_data).unwrap();
            } else if val_num != 0 {
                msg_channel.send(format!(
                    "Skipping line with unexpected number of values (expected {}, got {}): '{}'",
                    val_num,
                    values.len(),
                    line
                )).unwrap();
            }
        }
    }
    msg_channel
        .send("Standard input reader thread exiting.".to_string())
        .unwrap();
}
