use drone_consts::telemetry::Mode;

pub fn mode_to_labels(mode: Mode) -> Vec<&'static str> {
    match mode {
        Mode::None => Vec::new(),
        Mode::Imu => {
            vec!["gyr(x)", "gyr(y)", "gyr(z)", "acc(x)", "acc(y)", "acc(z)"]
        }
        Mode::Baro => vec!["altitude (m)"],
        Mode::Rc => vec![
            "Roll",
            "Pitch",
            "Throttle",
            "Yaw",
            "Kp Gain",
            "Kd Gain",
            "Arming",
            "Alt Hold",
            "Bbox Flush",
        ],
        Mode::Attitude => vec!["roll", "pitch", "yaw", "altitude"],
        Mode::Pid => vec![
            "roll",
            "pitch",
            "yaw",
            "altitude",
            "roll_i",
            "pitch_i",
            "yaw_i",
            "altitude_i",
        ],
        Mode::Mix | Mode::Dshot => vec![
            "M1(Front Right)",
            "M2(Back Left)",
            "M3(Front Left)",
            "M4(Back Right)",
        ],
    }
}
