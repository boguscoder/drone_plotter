use crossbeam_channel::{Receiver, Sender, bounded};
use drone_consts::telemetry::Category as TeleCategory;
use eframe::egui;
use egui::{CentralPanel, Color32, Panel, ScrollArea, Ui, ViewportBuilder};
use egui_plot::{Legend, Line, Plot};
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use std::{
    collections::VecDeque,
    time::{Duration, Instant},
};
use strum::IntoEnumIterator;

mod io;
mod utils;

const COLORS: [Color32; 9] = [
    Color32::from_rgb(255, 0, 0),     // Bright Red
    Color32::from_rgb(0, 255, 0),     // Lawn Green
    Color32::from_rgb(100, 170, 255), // Sky Blue
    Color32::from_rgb(255, 255, 0),   // Sunny Yellow
    Color32::from_rgb(255, 160, 0),   // Bright Orange
    Color32::from_rgb(200, 100, 255), // Bright Lavender
    Color32::from_rgb(0, 255, 255),   // Electric Cyan
    Color32::from_rgb(255, 128, 237), // Hot Pink
    Color32::from_rgb(255, 255, 255), // Pure White
];

const MAX_HISTORY_LEN: usize = 16384; // 2+ mins at 2KHz
const MAX_MSGS: usize = 16;

#[derive(Debug, Clone)]
struct SensorData {
    values: Vec<f64>,
    pub mode: TeleCategory,
}

struct Stats {
    frame_count: usize,
    msg_count: usize,
    last_update_time: Instant,
    frames_since_update: usize,
    frame_rate: f64,
}

#[derive(Debug, Clone)]
struct DataSeries {
    data: ConstGenericRingBuffer<[f64; 2], MAX_HISTORY_LEN>,
}

struct PlotterApp {
    data_history: Vec<DataSeries>,
    msg_history: VecDeque<(usize, String, Instant)>,
    data_receiver: Receiver<SensorData>,
    msg_receiver: Receiver<String>,
    tele_mode: TeleCategory,
    cmd_sender: Sender<TeleCategory>,
    stats: Stats,
    log_raw_data: bool,
    paused: bool,
}

impl PlotterApp {
    fn new(
        data_receiver: Receiver<SensorData>,
        msg_receiver: Receiver<String>,
        cmd_sender: Sender<TeleCategory>,
    ) -> Self {
        let mut app = Self {
            data_history: Vec::new(),
            msg_history: VecDeque::with_capacity(MAX_MSGS),
            data_receiver,
            msg_receiver,
            tele_mode: TeleCategory::None,
            cmd_sender,
            stats: Stats {
                frame_count: 0,
                msg_count: 0,
                last_update_time: Instant::now(),
                frames_since_update: 0,
                frame_rate: 0.0,
            },
            log_raw_data: false,
            paused: false,
        };
        app.apply_mode();
        app
    }

    fn apply_mode(&mut self) {
        while self.data_receiver.try_recv().is_ok() {}

        for series in &mut self.data_history {
            series.data.clear();
        }
        self.data_history = vec![
            DataSeries {
                data: ConstGenericRingBuffer::new(),
            };
            io::TELE_MAX_VALUES as usize
        ];
        self.cmd_sender.send(self.tele_mode).unwrap();
    }

    fn cleanup_messages(&mut self, now: Instant) -> bool {
        let initial_len = self.msg_history.len();

        while self.msg_history.len() > MAX_MSGS {
            self.msg_history.pop_front();
        }

        while self
            .msg_history
            .front()
            .is_some_and(|(_, _, time)| now.duration_since(*time) > Duration::from_secs(5))
        {
            self.msg_history.pop_front();
        }

        self.msg_history.len() != initial_len
    }

    fn drain_data(&mut self) -> bool {
        let mut batch_vals: Vec<Vec<f64>> = vec![Vec::new(); io::TELE_MAX_VALUES as usize];
        let mut updated = false;
        while let Ok(new_data) = self.data_receiver.try_recv() {
            if new_data.mode != self.tele_mode {
                continue;
            }

            self.stats.frame_count += 1;
            self.stats.frames_since_update += 1;

            let vals = new_data.values.len().min(io::TELE_MAX_VALUES as usize);
            if vals > 0 {
                for (batch_vec, &new_val) in batch_vals.iter_mut().zip(new_data.values.iter()) {
                    batch_vec.push(new_val);
                }
                if self.log_raw_data {
                    println!("Raw data: {:?}", new_data.values);
                }
            }
        }
        // per frame min/max downsampling
        if !batch_vals.is_empty() && !self.paused {
            let current_x = self.stats.frame_count as f64;

            for (i, series) in self.data_history.iter_mut().enumerate() {
                if batch_vals[i].is_empty() {
                    continue;
                }

                let mut min_val = f64::MAX;
                let mut max_val = f64::MIN;

                for &val in &batch_vals[i] {
                    min_val = min_val.min(val);
                    max_val = max_val.max(val);
                }

                series.data.enqueue([current_x, min_val]);
                series.data.enqueue([current_x, max_val]);
            }
            updated = true;
        }
        updated
    }

    fn ctrl_panel(&mut self, ui: &mut Ui) {
        Panel::top("control_panel").show(ui, |ui| {
            ui.add_space(2.0);
            ui.horizontal(|ui| {
                ui.heading(format!(
                    "Data Stream Rate: {:.2} msg/sec",
                    self.stats.frame_rate
                ));

                ui.add_space(ui.available_width() - 980.0);
                ui.add_space(ui.available_width() - 218.0);
                ui.add_space(10.0);
                ui.checkbox(&mut self.log_raw_data, "Log Data");

                let btn_text = if self.paused { "▶" } else { "⏸" };
                if ui.button(btn_text).clicked() {
                    self.paused = !self.paused;
                }

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
            ui.add_space(1.0);
        });
    }

    fn msg_panel(&mut self, ui: &mut Ui, now: Instant) -> bool {
        let mut updated = false;
        Panel::bottom("msg_panel").show(ui, |ui| {
            ui.add_space(5.0);
            for msg in self.msg_receiver.try_iter() {
                self.stats.msg_count += 1;
                println!("[{}] Message: {}", self.stats.msg_count, msg);
                self.msg_history.push_back((self.stats.msg_count, msg, now));
                updated = true;
            }

            if self.cleanup_messages(now) {
                updated = true;
            }

            if !self.msg_history.is_empty() {
                ScrollArea::vertical()
                    .max_width(f32::INFINITY)
                    .auto_shrink(false)
                    .max_height(50.0)
                    .show(ui, |ui| {
                        for (idx, msg, _) in self.msg_history.iter() {
                            ui.label(
                                egui::RichText::new(format!("[{}] {}", idx, msg))
                                    .color(egui::Color32::RED),
                            );
                        }
                    });
            } else {
                ui.label("No new messages.");
            }
            ui.add_space(5.0);
        });
        updated
    }

    fn plot(&mut self, ui: &mut Ui) {
        let labels = utils::mode_to_labels(self.tele_mode);

        CentralPanel::default()
            .frame(egui::Frame {
                fill: Color32::from_rgb(0x45, 0x5a, 0x64),
                ..Default::default()
            })
            .show(ui, |ui| {
                let msg_cap = self.stats.frame_rate * 5.0;
                let min_x = (self.stats.frame_count as f64 - msg_cap).max(0.0);

                Plot::new("telemetry_plot")
                    .legend(Legend::default())
                    .show_background(false)
                    .show_crosshair(self.paused)
                    .set_margin_fraction([0.01, 0.01].into())
                    .show(ui, |plot_ui| {
                        if !self.paused {
                            plot_ui.set_auto_bounds([true, true]);
                        }

                        for (i, (series, &label_name)) in
                            self.data_history.iter().zip(labels.iter()).enumerate()
                        {
                            if series.data.is_empty() {
                                continue;
                            }

                            let color = COLORS[i % COLORS.len()];

                            let points: Vec<[f64; 2]> = if !self.paused {
                                series
                                    .data
                                    .iter()
                                    .filter(|&&[x, _]| x >= min_x)
                                    .copied()
                                    .collect()
                            } else {
                                series.data.iter().copied().collect()
                            };

                            if points.is_empty() {
                                continue;
                            }

                            let line = Line::new(label_name, points).color(color);
                            plot_ui.line(line);
                        }
                    });
            });
    }
}

impl eframe::App for PlotterApp {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let mut has_data = self.drain_data();
        let elapsed_time = now.duration_since(self.stats.last_update_time);
        const RATE_UPDATE_INTERVAL: Duration = Duration::from_secs(1);

        if elapsed_time >= RATE_UPDATE_INTERVAL {
            self.stats.frame_rate =
                self.stats.frames_since_update as f64 / elapsed_time.as_secs_f64();
            self.stats.frames_since_update = 0;
            self.stats.last_update_time = now;
        }

        self.ctrl_panel(ui);
        has_data |= self.msg_panel(ui, now);
        self.plot(ui);

        if has_data {
            ui.request_repaint();
        }
    }
}

fn main() -> eframe::Result {
    let vp_builder: ViewportBuilder = ViewportBuilder::default();
    let options = eframe::NativeOptions {
        viewport: vp_builder.with_inner_size([1280.0, 720.0]),
        ..Default::default()
    };

    let (tx, rx) = bounded::<SensorData>(1024);
    let (etx, erx) = bounded::<String>(16);
    let (repaint_tx, repaint_rx) = bounded::<()>(16);
    let (cmd_tx, cmd_rx) = bounded::<TeleCategory>(16);

    io::start_input_threads(
        move || {
            let _ = repaint_tx.try_send(());
        },
        tx,
        etx,
        cmd_rx,
    );

    eframe::run_native(
        "Drone Stream Plotter",
        options,
        Box::new(move |cc| {
            let ctx = cc.egui_ctx.clone();
            std::thread::spawn(move || {
                while repaint_rx.recv().is_ok() {
                    ctx.request_repaint();
                }
            });
            Ok(Box::new(PlotterApp::new(rx, erx, cmd_tx)))
        }),
    )
}
