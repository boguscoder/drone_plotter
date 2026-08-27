use drone_consts::telemetry::Category as TeleCategory;

pub fn mode_to_labels(mode: TeleCategory) -> Vec<&'static str> {
    match mode {
        TeleCategory::None => Vec::new(),
        TeleCategory::Imu => vec!["gyr(x)", "gyr(y)", "gyr(z)", "acc(x)", "acc(y)", "acc(z)"],
        TeleCategory::Baro => vec!["altitude (m)"],
        TeleCategory::Rc => vec![
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
        TeleCategory::Attitude => vec!["roll", "pitch", "yaw", "altitude"],
        TeleCategory::Pid => vec![
            "roll",
            "pitch",
            "yaw",
            "altitude",
            "roll_i",
            "pitch_i",
            "yaw_i",
            "altitude_i",
        ],
        TeleCategory::Mix | TeleCategory::Dshot => vec![
            "M1(Front Right)",
            "M2(Back Left)",
            "M3(Front Left)",
            "M4(Back Right)",
        ],
        TeleCategory::Dump => vec![
            "Value 1", "Value 2", "Value 3", "Value 4", "Value 5", "Value 6", "Value 7", "Value 8",
        ],
    }
}
