use crossbeam_channel::{Receiver, Sender, bounded};
use drone_consts::telemetry::{Command, Mode};
use eframe::egui;
use egui::{CentralPanel, Color32, FontId, Panel, ScrollArea, Ui, ViewportBuilder};
use egui_plot::{HoverPosition, Legend, Line, Plot};
use ringbuffer::{AllocRingBuffer, RingBuffer};
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

const MAX_HISTORY_LEN: usize = 0x40000;
const MAX_MSGS: usize = 16;
const MAX_PLOT_POINTS: usize = 2048;

#[derive(Debug, Clone)]
struct SensorData {
    values: Vec<f32>,
    pub mode: Mode,
}

struct Stats {
    frame_count: usize,
    last_update_time: Instant,
    frames_since_update: usize,
    frame_rate: f64,
}

#[derive(Debug, Clone)]
struct DataSeries {
    data: AllocRingBuffer<[f64; 2]>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum DisplayState {
    Live,
    Paused { at_x: f64 },
    DumpView,
}

fn get_points(series: &DataSeries, state: DisplayState, data_rate: f64) -> Vec<[f64; 2]> {
    let len = series.data.len();
    if len == 0 {
        return Vec::new();
    }
    let (skip, take) = match state {
        DisplayState::Live => {
            let take = ((data_rate * 5.0).max(512.0).ceil() as usize).min(len);
            (len - take, take)
        }
        DisplayState::Paused { at_x } => match series.data.front() {
            Some(&[oldest, _]) => (0, ((at_x - oldest + 1.0).max(0.0) as usize).min(len)),
            None => (0, 0),
        },
        DisplayState::DumpView => (0, len),
    };
    if take == 0 {
        return Vec::new();
    }
    let stride = take.div_ceil(MAX_PLOT_POINTS);
    let mut it = series.data.iter().skip(skip).take(take);
    if stride <= 1 {
        return it.copied().collect();
    }
    let mut out = Vec::with_capacity(2 * MAX_PLOT_POINTS + 1);
    while let Some(&first) = it.next() {
        let (mut mn, mut mx) = (first, first);
        for &p in it.by_ref().take(stride - 1) {
            if p[1] < mn[1] {
                mn = p;
            }
            if p[1] > mx[1] {
                mx = p;
            }
        }
        if mn[0] <= mx[0] {
            out.push(mn);
            if mx != mn {
                out.push(mx);
            }
        } else {
            out.push(mx);
            out.push(mn);
        }
    }
    if let Some(&last) = series.data.get(skip + take - 1)
        && out.last() != Some(&last)
    {
        out.push(last);
    }
    out
}

struct PlotterApp {
    data_history: Vec<DataSeries>,
    msg_history: VecDeque<(String, Instant)>,
    data_receiver: Receiver<SensorData>,
    msg_receiver: Receiver<String>,
    tele_mode: Mode,
    cmd_sender: Sender<Command>,
    stats: Stats,
    log_raw_data: bool,
    state: DisplayState,
}

impl PlotterApp {
    fn new(
        data_receiver: Receiver<SensorData>,
        msg_receiver: Receiver<String>,
        cmd_sender: Sender<Command>,
    ) -> Self {
        let mut app = Self {
            data_history: Vec::new(),
            msg_history: VecDeque::with_capacity(MAX_MSGS),
            data_receiver,
            msg_receiver,
            tele_mode: Mode::Imu,
            cmd_sender,
            stats: Stats {
                frame_count: 0,
                last_update_time: Instant::now(),
                frames_since_update: 0,
                frame_rate: 0.0,
            },
            log_raw_data: false,
            state: DisplayState::Live,
        };
        app.apply_mode();
        app
    }

    fn cleanup_history(&mut self) {
        self.data_history = vec![
            DataSeries {
                data: AllocRingBuffer::new(MAX_HISTORY_LEN),
            };
            io::TELE_MAX_VALUES as usize
        ];
    }

    fn cleanup_messages(&mut self, now: Instant) -> bool {
        let initial_len = self.msg_history.len();

        while self.msg_history.len() > MAX_MSGS {
            self.msg_history.pop_front();
        }

        while self
            .msg_history
            .front()
            .is_some_and(|(_, time)| now.duration_since(*time) > Duration::from_secs(5))
        {
            self.msg_history.pop_front();
        }

        self.msg_history.len() != initial_len
    }

    fn apply_mode(&mut self) {
        while self.data_receiver.try_recv().is_ok() {}
        self.cleanup_history();
        self.cmd_sender
            .send(Command::SetTelemetryMode(self.tele_mode))
            .unwrap();
        self.state = DisplayState::Live;
    }

    fn drain_data(&mut self) -> bool {
        let mut updated = false;

        while let Ok(new_data) = self.data_receiver.try_recv() {
            if new_data.mode != self.tele_mode {
                continue;
            }

            self.stats.frame_count += 1;
            self.stats.frames_since_update += 1;

            let vals = new_data.values.len().min(io::TELE_MAX_VALUES as usize);
            if vals > 0 {
                for (i, &new_val) in new_data.values.iter().take(vals).enumerate() {
                    self.data_history[i]
                        .data
                        .enqueue([self.stats.frame_count as f64, new_val as f64]);
                }
                if self.log_raw_data {
                    println!("Raw data: {:?}", new_data.values);
                }
                updated = true;
            }
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

                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    egui::ComboBox::from_label("")
                        .selected_text(self.tele_mode.as_ref())
                        .show_ui(ui, |ui| {
                            for option in Mode::iter() {
                                if ui
                                    .selectable_value(&mut self.tele_mode, option, option.as_ref())
                                    .clicked()
                                {
                                    self.apply_mode();
                                }
                            }
                        });

                    if ui.button("💾").clicked() {
                        self.cleanup_history();
                        self.state = DisplayState::DumpView;
                        self.cmd_sender.send(Command::DumpFlash).unwrap();
                    }

                    let is_paused = matches!(self.state, DisplayState::Paused { .. });
                    let btn_text = if is_paused { "▶" } else { "⏸" };

                    if ui.button(btn_text).clicked() {
                        if is_paused {
                            self.state = DisplayState::Live;
                        } else {
                            self.state = DisplayState::Paused {
                                at_x: self.stats.frame_count as f64,
                            };
                        }
                    }

                    ui.checkbox(&mut self.log_raw_data, "Log Data");
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
                println!("{}", msg);
                self.msg_history.push_back((msg, now));
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
                        for (msg, _) in self.msg_history.iter() {
                            ui.label(
                                egui::RichText::new(msg)
                                    .color(egui::Color32::RED)
                                    .font(FontId::monospace(12.0)),
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
        let is_live = self.state == DisplayState::Live;

        CentralPanel::default()
            .frame(egui::Frame {
                fill: Color32::from_rgb(0x45, 0x5a, 0x64),
                ..Default::default()
            })
            .show(ui, |ui| {
                Plot::new("telemetry_plot")
                    .legend(Legend::default())
                    .show_background(false)
                    .show_crosshair(!is_live)
                    .set_margin_fraction([0.01, 0.01].into())
                    .label_formatter(|pos| match pos {
                        HoverPosition::NearDataPoint {
                            plot_name,
                            position,
                            ..
                        } if !plot_name.is_empty() => {
                            Some(format!("{}: {:}", plot_name, position.y))
                        }
                        _ => None,
                    })
                    .show(ui, |plot_ui| {
                        if !matches!(self.state, DisplayState::Paused { .. }) {
                            plot_ui.set_auto_bounds([true, true]);
                        }

                        for (i, (series, &label_name)) in
                            self.data_history.iter().zip(labels.iter()).enumerate()
                        {
                            if series.data.is_empty() {
                                continue;
                            }

                            let points: Vec<[f64; 2]> =
                                get_points(series, self.state, self.stats.frame_rate);

                            if points.is_empty() {
                                continue;
                            }

                            let line =
                                Line::new(label_name, points).color(COLORS[i % COLORS.len()]);
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
    let options = eframe::NativeOptions {
        viewport: ViewportBuilder::default()
            .with_inner_size([1280.0, 720.0])
            .with_icon(utils::load_app_icon()),
        ..Default::default()
    };

    let (tx, rx) = bounded::<SensorData>(1024);
    let (etx, erx) = bounded::<String>(16);
    let (repaint_tx, repaint_rx) = bounded::<()>(16);
    let (cmd_tx, cmd_rx) = bounded::<Command>(16);

    io::start_input_threads(
        move || {
            let _ = repaint_tx.try_send(());
        },
        tx,
        etx,
        cmd_rx,
    );

    eframe::run_native(
        "Drone Telemetry Plotter",
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
