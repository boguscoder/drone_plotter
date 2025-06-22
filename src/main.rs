use core::sync::atomic::{AtomicUsize, Ordering};
use crossbeam_channel::{Receiver, unbounded};
use eframe::egui;
use egui::{CentralPanel, ScrollArea, containers::TopBottomPanel};
use egui::{Color32, ViewportBuilder};
use egui_plotter::EguiBackend;
use plotters::prelude::full_palette::*;
use plotters::prelude::*;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use serialport::SerialPort;
use std::io::Write;
use std::time::{Duration, Instant};
use strum::IntoEnumIterator;
use strum_macros::{AsRefStr, EnumIter};

mod io;

static VALS_PER_LINE: AtomicUsize = AtomicUsize::new(0);

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
const MAX_MSGS: usize = 16;

// TODO: keep in sync with simplest_drone, move to shared crate one day
#[derive(Debug, EnumIter, AsRefStr, PartialEq, Clone, Copy)]
enum TeleCategory {
    None = 0,
    Imu,
    Attitude,
    Pid,
    Mix,
    Dshot,
}

#[derive(Debug, Clone)]
struct SensorData {
    values: Vec<f64>,
}

struct Stats {
    lines_count: usize,
    last_update_time: Instant,
    lines_since_update: usize,
    line_rate: f64,
}

struct PlotterApp {
    data_history: Vec<ConstGenericRingBuffer<f64, MAX_HISTORY_LEN>>,
    msg_history: ConstGenericRingBuffer<String, MAX_MSGS>,
    data_receiver: Receiver<SensorData>,
    msg_receiver: Receiver<String>,
    tele_mode: TeleCategory,
    tele_port: Box<dyn SerialPort>,
    stats: Stats,
}

impl PlotterApp {
    fn new(
        data_receiver: Receiver<SensorData>,
        msg_receiver: Receiver<String>,
        tele_port: Box<dyn SerialPort>,
    ) -> Self {
        let mut app = Self {
            data_history: Vec::new(),
            msg_history: ConstGenericRingBuffer::default(),
            data_receiver,
            msg_receiver,
            tele_mode: TeleCategory::None,
            tele_port,
            stats: Stats {
                lines_count: 0,
                last_update_time: Instant::now(),
                lines_since_update: 0,
                line_rate: 0.0,
            },
        };
        app.apply_mode();
        app
    }

    fn mode_to_dim(mode: TeleCategory) -> usize {
        match mode {
            TeleCategory::None => 0,
            TeleCategory::Imu => 9,
            TeleCategory::Attitude => 3,
            TeleCategory::Pid => 4,
            TeleCategory::Mix => 4,
            TeleCategory::Dshot => 4,
        }
    }

    fn mode_to_labels(mode: TeleCategory) -> Vec<&'static str> {
        match mode {
            TeleCategory::None => Vec::new(),
            TeleCategory::Imu => vec![
                "gyr(x)", "gyr(y)", "gyr(z)", "acc(x)", "acc(y)", "acc(z)", "mag(x)", "mag(y)",
                "mag(z)",
            ],
            TeleCategory::Attitude => vec!["roll", "pitch", "yaw"],
            TeleCategory::Pid => vec!["throttle", "roll", "pitch", "yaw"],
            TeleCategory::Mix => vec!["M1", "M2", "M3", "M4"],
            TeleCategory::Dshot => vec!["M1", "M2", "M3", "M4"],
        }
    }

    fn apply_mode(&mut self) {
        let new_dim = Self::mode_to_dim(self.tele_mode);
        VALS_PER_LINE.store(new_dim, Ordering::Release);
        self.data_history = vec![ConstGenericRingBuffer::new(); new_dim];
        self.tele_port
            .write_all(format!("{}\n", self.tele_mode as u8).as_bytes())
            .unwrap();
    }
}

impl eframe::App for PlotterApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();

        while let Ok(new_data) = self.data_receiver.try_recv() {
            let vals = VALS_PER_LINE.load(Ordering::Acquire);
            if vals > 0 {
                self.stats.lines_count += 1;
                for i in 0..vals {
                    self.data_history[i].enqueue(new_data.values[i]);
                }
                self.stats.lines_since_update += 1;
            }
        }

        let elapsed_time = now.duration_since(self.stats.last_update_time);
        const RATE_UPDATE_INTERVAL: Duration = Duration::from_secs(1);

        if elapsed_time >= RATE_UPDATE_INTERVAL {
            self.stats.line_rate =
                self.stats.lines_since_update as f64 / elapsed_time.as_secs_f64();
            self.stats.lines_since_update = 0;
            self.stats.last_update_time = now;
        }

        TopBottomPanel::top("mode_panel").show(ctx, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.heading(format!(
                    "Data Stream Rate: {:.2} lines/sec",
                    self.stats.line_rate
                ));
                ui.add_space(ui.available_width() - 100.0);
                egui::ComboBox::from_label("")
                    .selected_text(self.tele_mode.as_ref())
                    .show_ui(ui, |ui| {
                        for option in TeleCategory::iter() {
                            if ui
                                .selectable_value(&mut self.tele_mode, option, option.as_ref())
                                .clicked()
                            {
                                self.apply_mode();
                            }
                        }
                    });
            });
            ui.add_space(2.0);
        });

        TopBottomPanel::bottom("msg_panel").show(ctx, |ui| {
            ui.add_space(5.0);
            self.msg_history.extend(self.msg_receiver.try_iter());
            if !self.msg_history.is_empty() {
                ScrollArea::vertical()
                    .max_width(f32::INFINITY)
                    .auto_shrink(false)
                    .max_height(50.0)
                    .show(ui, |ui| {
                        for msg in self.msg_history.iter() {
                            ui.label(egui::RichText::new(msg).color(egui::Color32::RED));
                        }
                    });
            } else {
                ui.label("No messages.");
            }
            ui.add_space(5.0);
        });

        CentralPanel::default().show(ctx, |ui| {
            egui::Frame::canvas(ui.style())
                .fill(Color32::from_white_alpha(0))
                .stroke(egui::Stroke::NONE)
                .show(ui, |ui_plot| {
                    let root = EguiBackend::new(ui_plot).into_drawing_area();
                    root.fill(&BLUEGREY_700).unwrap();

                    // Define X-axis range (based on sample count).
                    let min_x = (self.stats.lines_count as f64 - MAX_HISTORY_LEN as f64).max(0.0);
                    let max_x = self.stats.lines_count as f64;

                    let has_data =
                        !self.data_history.is_empty() && !self.data_history[0].is_empty();
                    // Determine Y-axis range for auto-scaling.
                    let mut min_y = if has_data { f64::MAX } else { 0.0 };
                    let mut max_y = if has_data { f64::MIN } else { 0.0 };
                    for series_data in &self.data_history {
                        for &val in series_data.iter() {
                            min_y = min_y.min(val);
                            max_y = max_y.max(val);
                        }
                    }

                    let mut chart = ChartBuilder::on(&root)
                        .x_label_area_size(30)
                        .y_label_area_size(40)
                        .build_cartesian_2d(min_x..max_x, min_y..max_y)
                        .unwrap();

                    chart.configure_mesh().draw().unwrap();

                    let labels = Self::mode_to_labels(self.tele_mode);
                    for (i, series_data) in self.data_history.iter().enumerate() {
                        let series_points: Vec<(f64, f64)> = series_data
                            .iter()
                            .enumerate()
                            .map(|(j, &val)| {
                                ((self.stats.lines_count - series_data.len() + j) as f64, val)
                            })
                            .collect();

                        chart
                            .draw_series(LineSeries::new(series_points, COLORS[i].filled()))
                            .unwrap()
                            .label(labels[i])
                            .legend(move |(x, y)| {
                                PathElement::new(vec![(x, y), (x + 20, y)], COLORS[i].filled())
                            });
                    }

                    chart.configure_series_labels().draw().unwrap();
                });
        });

        ctx.request_repaint();
    }
}

fn main() -> eframe::Result {
    let vp_builder: ViewportBuilder = ViewportBuilder::default();
    let options = eframe::NativeOptions {
        viewport: vp_builder.with_inner_size([1280.0, 720.0]),
        ..Default::default()
    };

    let (tx, rx) = unbounded::<SensorData>();
    let (etx, erx) = unbounded::<String>();

    std::thread::spawn(move || {
        io::input_thread(tx, etx);
    });

    let tele_port = match io::open_out_port() {
        Ok(port) => port,
        Err(e) => {
            return eframe::Result::Err(eframe::Error::AppCreation(Box::new(e)));
        }
    };

    eframe::run_native(
        "Drone Stream Plotter",
        options,
        Box::new(move |_cc| Ok(Box::new(PlotterApp::new(rx, erx, tele_port)))),
    )
}
