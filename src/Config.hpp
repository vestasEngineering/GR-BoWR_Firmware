#pragma once
#include <Arduino.h>

// -------------------------
// Centralized configuration
// -------------------------

struct ActuatorCfg {
    float maxCommandVoltage       = 5.0f;
    float maxFeedbackVoltage      = 5.0f;

    // Factory actuator stroke command used by blade transitions.
    // Runtime configuration may override this value after startup.
    float transitionActiveVoltage = 4.4f;

    uint16_t settle_ms            = 15000;
    float activate_cmd_min_v      = 4.0f;
    float deactivate_cmd_max_v    = 0.5f;
    float active_fb_max_v         = 1.00f;
    float inactive_fb_min_v       = 2.20f;

    float fb_map_m                = -0.529920101f;
    float fb_map_b                =  2.705734968f;
};

struct AndonCfg {
    int      pin                 = D4;     // WS2812 data line (driven directly)
    uint16_t num_leds            = 41;
    uint8_t  brightness          = 255;    // global 0-255 scale
    uint16_t frame_dt_ms         = 16;     // ~60 Hz
    uint16_t blink_interval_ms   = 500;
    uint16_t boot_duration_ms    = 5000;
    uint16_t fade_ms             = 350;
};

struct AndonManagerCfg {
    uint16_t tick_period_ms      = 100;
    uint16_t stable_ms           = 200;
    uint16_t comms_grace_ms      = 3000;

    uint16_t ut_verify_window_ms = 3000;
    float    ut_ok_min_distance  = 60.0f;
    uint16_t ut_good_consec      = 3;
    uint16_t ultrasonic_bad_streak_threshold = 20;
};

struct BatteryCfg {
    float vref                  = 3.1f;
    float adc_max_counts        = 1023.0f;
    float r1_ohm                = 100000.0f;
    float r2_ohm                = 10000.0f;

    float max_voltage           = 20.0f;
    float min_voltage           = 16.5f;

    uint8_t oled_width          = 128;
    uint8_t oled_height         = 64;
    int     oled_reset_pin      = -1;
    uint8_t oled_i2c_addr       = 0x3D;

    uint32_t emit_ms            = 100000;
    float    emit_pct_delta     = 1.0f;

    float    low_pct            = 20.0f;
    float    critical_pct       = 10.0f;
};


struct EStopCfg {
    int      pin                = D0;
    bool     active_high        = false;
    uint8_t  pull_mode          = 2;      // 0=None, 1=Pullup, 2=Pulldown
    bool     latch              = false;  // when true, ACTIVE latches until cleared in software
};

struct MotorsCfg {
    // --- Pins (ClearPath step & direction, one set per motor) ---
    // Axis 0 = "motor 1" connector, axis 1 = "motor 2" connector.
    int pin_hlfb[2]             = { D8,  D14 };
    int pin_step[2]             = { D5,  D9  };
    int pin_dir[2]              = { D6,  D21  };
    int pin_enable[2]           = { D7,  D3  };   // D11/D12 are I2C (Wire) - do not use

    bool enable_active_high     = true;
    uint32_t enable_settle_ms   = 500;      // ENABLE asserted -> first STEP pulse

    // Which axis drives the left / right track (when facing forward).
    bool axis0_is_left          = true;

    // +1 / -1: sign applied to a forward (positive) speed to get the DIR pin
    // level. Tracks are mirrored, so one axis is normally inverted.
    int8_t motor_direction[2]   = { 1, 1 };
    // +1 / -1: sign applied to HLFB pulse counts so forward travel is positive.
    int8_t encoder_direction[2] = { 1, 1 };

    // --- Mechanics ---
    float wheel_diameter_m      = 0.185f;   // main drive wheel
    float gearbox_ratio         = 10.0f;    // motor revs per wheel rev
    uint32_t steps_per_motor_rev = 3200;    // ClearPath step resolution setting

    // --- HLFB used as encoder (16 pulses per motor rev, no direction) ---
    uint16_t hlfb_ppr           = 16;
    // Displacement per HLFB pulse. Default = pi * D / (gear * PPR) = 3.632 mm.
    // Tune this against a measured distance to calibrate odometry.
    float hlfb_mm_per_pulse     = 3.14159265f * 185.0f / (10.0f * 16.0f);
    // Minimum time between accepted HLFB edges (glitch filter), microseconds.
    uint32_t hlfb_min_edge_us   = 200;

    // --- Step generation ---
    uint16_t step_pulse_ticks   = 1;        // STEP high time in 10 us ISR ticks
    float max_speed_ms          = 0.30f;    // hard clamp on any wheel speed
    float accel_mps2            = 1.0f;     // slew limit for speed increases
    float decel_mps2            = 2.0f;     // slew limit for speed decreases
    float brake_decel_mps2      = 4.0f;     // BRAKE_STOP ramp

    // Encoders considered "stopped" below this wheel speed
    float zero_speed_ms         = 0.0002f;
};

struct SteeringCfg {
    bool     enabled            = true;

    // Laser-line angle (deg) is regulated to this value; 0 = drive parallel.
    float    target_angle_deg   = 0.0f;

    // +1 : positive angle means the robot must turn toward the right
    //      (left track faster). -1 inverts it.
    int8_t   sign               = 1;

    // Angle -> yaw rate (rad/s per rad) and integral term.
    float    kp                 = 1.5f;
    float    ki                 = 0.0f;
    float    integral_limit_rad_s = 0.20f;
    float    deadband_deg       = 0.1f;

    // Distance between the two track centerlines.
    float    track_width_m      = 0.30f;

    // Largest speed difference between the tracks.
    float    max_diff_ms        = 0.04f;

    // Below this mean speed steering is not applied (and the integral resets).
    float    min_active_speed_ms = 0.005f;

    // Angle freshness.
    uint16_t stale_ms           = 250;      // age after which an angle is ignored
    uint16_t startup_grace_ms   = 3000;     // wait for first angle after process start
    bool     stop_process_when_stale = true;
};

struct JogCfg {
    int  pin_forward            = D1;
    int  pin_backward           = D10;
    float jog_speed_max_ms      = 0.05f;
    uint16_t jog_duration_ms    = 1000;
};

struct UltrasonicCfg {
    uint8_t analog_pin           = A2;

    float mm_per_volt            = (300.0f - 30.0f) / (2.0f - 0.0f);
    float offset_mm              = 40.0f;

    float valid_min_mm           = 45.0f;
    float valid_max_mm           = 250.0f;

    float ignored_close_min_mm   = 38.0f;
    float ignored_close_max_mm   = 47.0f;

    float control_loop_dt_s      = 0.01f;

    float motion_accel_mps2      = 0.15f;
    float motion_decel_mps2      = 0.30f;

    float setpoint_mm            = 165.0f;
    
    float base_speed_ms          = 0.095f; // Feed-forward base speed: 0.095 m/s = 95 mm/s.

    float kp_speed_up_track      = 0.00025f;
    float kp_slow_down_track     = 0.0020f;

    float max_speed_ms           = 0.25f;

    float min_track_speed_ms     = 0.035f;
    float max_track_speed_ms     = 0.120f;

    float tracking_deadband_mm   = 3.0f;

    float wait_range_mm          = 25.0f;

    uint8_t good_glue_required   = 3;

    float track_accel_mps2       = 0.25f;
    float track_decel_mps2       = 1.00f;

    bool telemetry_enabled       = false;
    uint16_t telemetry_period_ms = 50; // 20 Hz
};

struct SerialCfg {
    uint16_t rx_keepalive_ms    = 2000;
    uint16_t rx_led_flash_ms    = 50;
    size_t   json_rx_capacity   = 640;
    size_t   json_tx_capacity   = 640;
    size_t   rx_buf_bytes       = 512;
};

struct ContactorCfg {
    int pin = D2;
    bool high_means_power_lost = true;
    bool pullup = true;
    uint16_t debounce_ms = 75;
};

struct RobotConfig {
    ActuatorCfg        actuator;
    AndonCfg           andon;
    AndonManagerCfg    andonMgr;
    BatteryCfg         battery;
    EStopCfg           estop;
    MotorsCfg          motors;
    JogCfg             jog;
    UltrasonicCfg      ultrasonic;
    SteeringCfg        steering;
    SerialCfg          serial;
    ContactorCfg       contactor;
};

extern RobotConfig CFG;