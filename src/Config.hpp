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
    uint8_t  neo_addr            = 0x61;
    uint16_t num_leds            = 41;
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


struct ClampCfg {
    int       pin                 = D6;      // TLP785 output into Portenta H7
    bool      active_high         = true;    // HIGH = clamped OK
    uint16_t  debounce_ms         = 20;      // input debounce
    uint32_t  emit_ms             = 1000;    // optional telemetry cadence (if you want to emit)
};


struct EStopCfg {
    int      pin                = D0;
    bool     active_high        = false;
    uint8_t  pull_mode          = 2;      // 0=None, 1=Pullup, 2=Pulldown
    bool     latch              = false;  // when true, ACTIVE latches until cleared in software
};

struct MotorsCfg {
    uint32_t can_bitrate        = 250000;
    uint16_t apos_hz            = 50;
    bool     apos_query_all     = false;

    float wheel_diameter_m      = 0.048f;
    int   encoder_cpr           = 4096;
    float encoder_to_mm         = 310.0f / 280.0f;

    float erefs_k               = 18.7187185f;

    uint32_t apos_ids[4]        = { 0x16002005, 0x16004005, 0x16006005, 0x16008005 };
    uint32_t erefs_ids[4]       = { 0x048020A8, 0x048040A8, 0x048060A8, 0x048080A8 };
    uint32_t reset_apos_ids[4]  = { 0x00802002, 0x00804002, 0x00806002, 0x00808002 };
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

struct UltrasonicServoCfg {
    int pwm_pin                 = D3;
    int active_angle_deg        = 00;
    int inactive_angle_deg      = 170;
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
    UltrasonicServoCfg ut_servo;
    SerialCfg          serial;
    ClampCfg           clamp;
    ContactorCfg       contactor;
};

extern RobotConfig CFG;