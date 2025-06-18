use crossbeam_channel::{Receiver, Sender, unbounded};
use eframe::egui;
use egui::ViewportBuilder;
use egui_plotter::EguiBackend;
use plotters::prelude::full_palette::*;
use plotters::prelude::*;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use std::io::{self, BufRead, BufReader};
use std::time::{Duration, Instant}; // Add these imports

// log_imu
#[cfg(feature = "log_imu")]
const VALS_PER_LINE: usize = 9;
#[cfg(feature = "log_imu")]
const LABELS: [&str; VALS_PER_LINE] = [
    "gyr(x)", "gyr(y)", "gyr(z)", "acc(x)", "acc(y)", "acc(z)", "mag(x)", "mag(y)", "mag(z)",
];

// log_att
#[cfg(feature = "log_att")]
const VALS_PER_LINE: usize = 3;
#[cfg(feature = "log_att")]
const LABELS: [&str; VALS_PER_LINE] = ["roll", "pitch", "yaw"];

//log_pid
#[cfg(feature = "log_pid")]
const VALS_PER_LINE: usize = 4;
#[cfg(feature = "log_pid")]
const LABELS: [&str; VALS_PER_LINE] = ["throttle", "roll", "pitch", "yaw"];

// log_dshot
#[cfg(any(feature = "log_dshot", feature = "log_mix"))]
const VALS_PER_LINE: usize = 4;
#[cfg(any(feature = "log_dshot", feature = "log_mix"))]
const LABELS: [&str; VALS_PER_LINE] = ["M1", "M2", "M3", "M4"];

const COLORS: [plotters::style::RGBColor; 9] = [
    RGBColor(141, 215, 242),
    RGBColor(78, 205, 196),
    RGBColor(179, 157, 219),
    RGBColor(255, 224, 102),
    RGBColor(255, 168, 0),
    RGBColor(255, 107, 107),
    RGBColor(167, 242, 110),
    RGBColor(255, 128, 237),
    RGBColor(128, 222, 234),
];
const MAX_HISTORY_LEN: usize = 512;

#[derive(Debug, Clone)]
struct SensorData {
    values: [f64; VALS_PER_LINE],
}

struct PlotterApp {
    data_history: [ConstGenericRingBuffer<f64, MAX_HISTORY_LEN>; VALS_PER_LINE],
    lines_count: usize,
    data_receiver: Receiver<SensorData>,
    last_update_time: Instant,
    lines_since_update: usize,
    line_rate: f64,
}

impl PlotterApp {
    fn new(data_receiver: Receiver<SensorData>) -> Self {
        Self {
            data_history: [const { ConstGenericRingBuffer::<f64, MAX_HISTORY_LEN>::new() };
                VALS_PER_LINE],
            lines_count: 0,
            data_receiver,
            last_update_time: Instant::now(),
            lines_since_update: 0,
            line_rate: 0.0,
        }
    }
}

impl eframe::App for PlotterApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();

        while let Ok(new_data) = self.data_receiver.try_recv() {
            self.lines_count += 1;
            for i in 0..VALS_PER_LINE {
                self.data_history[i].enqueue(new_data.values[i]);
            }
            self.lines_since_update += 1;
            ctx.request_repaint();
        }

        let elapsed_time = now.duration_since(self.last_update_time);
        const RATE_UPDATE_INTERVAL: Duration = Duration::from_secs(1);

        if elapsed_time >= RATE_UPDATE_INTERVAL {
            self.line_rate = self.lines_since_update as f64 / elapsed_time.as_secs_f64();
            self.lines_since_update = 0;
            self.last_update_time = now;
            ctx.request_repaint();
        }

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading(format!("Data Stream Rate: {:.2} lines/sec", self.line_rate));
            ui.separator();
            ui.add_space(5.0);

            egui::Frame::canvas(ui.style()).show(ui, |ui_plot| {
                let root = EguiBackend::new(ui_plot).into_drawing_area();
                root.fill(&BLUEGREY_700).unwrap();

                // Define X-axis range (based on sample count).
                let min_x = (self.lines_count as f64 - MAX_HISTORY_LEN as f64).max(0.0);
                let max_x = self.lines_count as f64;

                // Determine Y-axis range for auto-scaling.
                let mut min_y = f64::MAX;
                let mut max_y = f64::MIN;
                for series_data in &self.data_history {
                    for &val in series_data.iter() {
                        min_y = min_y.min(val);
                        max_y = max_y.max(val);
                    }
                }

                let mut chart = ChartBuilder::on(&root)
                    .margin(10)
                    .x_label_area_size(30)
                    .y_label_area_size(40)
                    .build_cartesian_2d(min_x..max_x, min_y..max_y)
                    .unwrap();

                chart.configure_mesh().draw().unwrap();

                for (i, series_data) in self.data_history.iter().enumerate() {
                    let series_points: Vec<(f64, f64)> = series_data
                        .iter()
                        .enumerate()
                        .map(|(j, &val)| ((self.lines_count - series_data.len() + j) as f64, val))
                        .collect();

                    chart
                        .draw_series(LineSeries::new(series_points, COLORS[i].filled()))
                        .unwrap()
                        .label(LABELS[i])
                        .legend(move |(x, y)| {
                            PathElement::new(vec![(x, y), (x + 20, y)], COLORS[i].filled())
                        });
                }

                chart.configure_series_labels().draw().unwrap();
            });
        });
    }
}

fn stdin_reader_thread(sender: Sender<SensorData>) {
    let stdin = io::stdin();
    let reader = BufReader::new(stdin.lock());

    for line_result in reader.lines() {
        match line_result {
            Ok(line) => {
                let values: Vec<f64> = line
                    .split(',')
                    .filter_map(|s| {
                        s.parse().ok().and_then(|val: f64| {
                            if val.is_finite() {
                                Some(val)
                            } else {
                                eprintln!("Skipping non-finite value: {}", s);
                                None
                            }
                        })
                    })
                    .collect();

                if values.len() == VALS_PER_LINE {
                    let mut sensor_data = SensorData {
                        values: [0.0; VALS_PER_LINE],
                    };
                    for (i, &val) in values.iter().enumerate() {
                        sensor_data.values[i] = val;
                    }
                    if let Err(e) = sender.send(sensor_data) {
                        eprintln!("Failed to send data to GUI thread: {}", e);
                        break;
                    }
                } else {
                    eprintln!(
                        "Skipping line with unexpected number of values (expected {}, got {}): '{}'",
                        VALS_PER_LINE,
                        values.len(),
                        line
                    );
                }
            }
            Err(e) => {
                eprintln!("Error reading from stdin: {}", e);
                break;
            }
        }
    }
    println!("Standard input reader thread exiting.");
}

fn main() -> eframe::Result {
    let vp_builder: ViewportBuilder = ViewportBuilder::default();
    let options = eframe::NativeOptions {
        viewport: vp_builder.with_inner_size([1280.0, 720.0]),
        ..Default::default()
    };

    let (tx, rx) = unbounded::<SensorData>();
    let sender_for_thread = tx.clone();

    std::thread::spawn(move || {
        stdin_reader_thread(sender_for_thread);
    });

    eframe::run_native(
        "Drone Stream Plotter",
        options,
        Box::new(move |_cc| Ok(Box::new(PlotterApp::new(rx)))),
    )
}
