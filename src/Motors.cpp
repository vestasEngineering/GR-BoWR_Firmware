#include "Motors.hpp"
#include <math.h>

const int8_t Motors::AXIS_SIGN[4] = { 1, -1, 1, -1 };

// ---------- Constructors ----------
Motors::Motors()
    : rcA(&roboclaw_uart_a, 10000),
      rcB(&roboclaw_uart_b, 10000) {}

Motors::Motors(UART* ua, UART* ub)
    : rcA(ua, 10000),
      rcB(ub, 10000) {}


// ---------- Internal helpers ----------
uint32_t Motors::u32bits(int32_t v) {
    // Preserve the exact 32-bit two's-complement bit pattern expected by the controller.
    return static_cast<uint32_t>(v);
}

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


// ---------- Init ----------
void Motors::begin() {
    rcA.begin(BAUD);
    rcB.begin(BAUD);
    delay(50);

    // Stop if serial commands stop arriving for > 200 ms.
    rcA.SetTimeout(ADDR_A, 2.0f);
    rcB.SetTimeout(ADDR_B, 2.0f);

    STOP();
    delay(10);
    resetEncoders();
}

int32_t Motors::ms_to_qpps(float ms) const {
    // Guard against NaN/Inf from upstream code.
    if (!isfinite(ms)) {
        ms = 0.0f;
    }

    // Ignore very small commands so sensor jitter / PID residue does not create creep.
    if (fabsf(ms) < ZERO_SPEED_THRESHOLD_MS) {
        return 0;
    }

    const float wheelCirc = PI * WHEEL_DIAMETER_M;
    const float revsPerSec = ms / wheelCirc;
    return static_cast<int32_t>(lroundf(revsPerSec * ENCODER_CPR));
}


// ---------- Send commands ----------
void Motors::setSpeeds(float s0, float s1, float s2, float s3) {
    noInterrupts();
    speeds[0] = s0;
    speeds[1] = s1;
    speeds[2] = s2;
    speeds[3] = s3;
    interrupts();
}

void Motors::sendSpeeds() {
    if (commandHz == 0) {
        return;
    }

    uint32_t period = 1000UL / commandHz;
    if (period == 0) {
        period = 1;
    }

    const uint32_t now = millis();
    if (now - lastSendMs < period) {
        return;
    }
    lastSendMs = now;

    int32_t q0 = clamp_i32(ms_to_qpps(speeds[0]), -maxCommandQpps, maxCommandQpps);
    int32_t q1 = clamp_i32(ms_to_qpps(speeds[1]), -maxCommandQpps, maxCommandQpps);
    int32_t q2 = clamp_i32(ms_to_qpps(speeds[2]), -maxCommandQpps, maxCommandQpps);
    int32_t q3 = clamp_i32(ms_to_qpps(speeds[3]), -maxCommandQpps, maxCommandQpps);

    const uint32_t a_m1 = u32bits(q0);
    const uint32_t a_m2 = u32bits(-q1);
    const uint32_t b_m1 = u32bits(-q2);
    const uint32_t b_m2 = u32bits(q3);

    // If everything is commanded to zero, send raw zero duty instead of speed=0.
    if (q0 == 0 && q1 == 0 && q2 == 0 && q3 == 0) {
        rcA.DutyM1M2(ADDR_A, 0, 0);
        rcB.DutyM1M2(ADDR_B, 0, 0);
        return;
    }

    rcA.SpeedAccelM1M2(ADDR_A, ACCEL_QPPS_S, a_m1, a_m2);
    rcB.SpeedAccelM1M2(ADDR_B, ACCEL_QPPS_S, b_m1, b_m2);
}


// ---------- Read feedback ----------
void Motors::pollEncoders() {
    if (pollHz == 0) {
        return; // polling disabled
    }

    uint32_t period = 1000UL / pollHz;
    if (period == 0) {
        period = 1; // guard very high pollHz values
    }

    const uint32_t now = millis();
    if (now - lastPollMs < period) {
        return;
    }
    lastPollMs = now;

    uint8_t st = 0;
    bool ok = false;

    // --- Encoders ---
    {
        const int32_t v = static_cast<int32_t>(rcA.ReadEncM1(ADDR_A, &st, &ok));
        encStatus_[0] = st;
        encValid_[0] = ok;
        if (ok) encCounts[0] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcA.ReadEncM2(ADDR_A, &st, &ok));
        encStatus_[1] = st;
        encValid_[1] = ok;
        if (ok) encCounts[1] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcB.ReadEncM1(ADDR_B, &st, &ok));
        encStatus_[2] = st;
        encValid_[2] = ok;
        if (ok) encCounts[2] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcB.ReadEncM2(ADDR_B, &st, &ok));
        encStatus_[3] = st;
        encValid_[3] = ok;
        if (ok) encCounts[3] = v;
    }

    // --- Speeds ---
    {
        const int32_t v = static_cast<int32_t>(rcA.ReadSpeedM1(ADDR_A, &st, &ok));
        speedStatus_[0] = st;
        speedValid_[0] = ok;
        if (ok) qpps[0] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcA.ReadSpeedM2(ADDR_A, &st, &ok));
        speedStatus_[1] = st;
        speedValid_[1] = ok;
        if (ok) qpps[1] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcB.ReadSpeedM1(ADDR_B, &st, &ok));
        speedStatus_[2] = st;
        speedValid_[2] = ok;
        if (ok) qpps[2] = v;
    }

    {
        const int32_t v = static_cast<int32_t>(rcB.ReadSpeedM2(ADDR_B, &st, &ok));
        speedStatus_[3] = st;
        speedValid_[3] = ok;
        if (ok) qpps[3] = v;
    }
}


// ---------- Public loop ----------
void Motors::update() {
    sendSpeeds();
    pollEncoders();
}


// ---------- Safety ----------
void Motors::STOP() {
    for (float &s : speeds) {
        s = 0.0f;
    }

    rcA.DutyM1M2(ADDR_A, 0, 0);
    rcB.DutyM1M2(ADDR_B, 0, 0);
}


// ---------- Maintenance ----------
void Motors::resetEncoders() {
    rcA.ResetEncoders(ADDR_A);
    rcB.ResetEncoders(ADDR_B);
    delay(20);

    for (int i = 0; i < 4; ++i) {
        encCounts[i] = 0;
        qpps[i] = 0;
        encStatus_[i] = 0;
        speedStatus_[i] = 0;
        encValid_[i] = false;
        speedValid_[i] = false;
    }
}


// ---------- Calibrated distance helpers ----------

int32_t Motors::getNormalizedCounts(uint8_t axis) const {
    if (axis >= 4) {
        return 0;
    }

    return encCounts[axis] * AXIS_SIGN[axis];
}

float Motors::getWheelMM(uint8_t axis) const {
    if (axis >= 4) {
        return 0.0f;
    }

    return static_cast<float>(getNormalizedCounts(axis)) * DISTANCE_MM_PER_COUNT;
}

float Motors::getRobotRearDistanceMM() const {
    // Rear wheels are axis 2 and axis 3.
    // Axis 3 is sign-normalized before averaging.
    return 0.5f * (getWheelMM(2) + getWheelMM(3));
}

float Motors::getRobotRearDistanceM() const {
    return getRobotRearDistanceMM() / 1000.0f;
}

int32_t Motors::mm_to_distance_counts(float mm) const {
    if (!isfinite(mm)) {
        mm = 0.0f;
    }

    return static_cast<int32_t>(lroundf(mm / DISTANCE_MM_PER_COUNT));
}

void Motors::setRobotRearDistanceMM(float mm) {
    if (!isfinite(mm)) {
        mm = 0.0f;
    }

    const int32_t normalizedCounts = mm_to_distance_counts(mm);

    const int32_t raw0 = normalizedCounts * AXIS_SIGN[0];
    const int32_t raw1 = normalizedCounts * AXIS_SIGN[1];
    const int32_t raw2 = normalizedCounts * AXIS_SIGN[2];
    const int32_t raw3 = normalizedCounts * AXIS_SIGN[3];

    // Stop before forcing encoder values.
    STOP();
    delay(10);

    rcA.SetEncM1(ADDR_A, u32bits(raw0));
    rcA.SetEncM2(ADDR_A, u32bits(raw1));
    rcB.SetEncM1(ADDR_B, u32bits(raw2));
    rcB.SetEncM2(ADDR_B, u32bits(raw3));

    delay(20);

    // Update local cache immediately so the HMI reflects the set value
    // without waiting for the next poll.
    encCounts[0] = raw0;
    encCounts[1] = raw1;
    encCounts[2] = raw2;
    encCounts[3] = raw3;

    for (int i = 0; i < 4; ++i) {
        qpps[i] = 0;
    }
}

void Motors::setRobotRearDistanceM(float meters) {
    if (!isfinite(meters)) {
        meters = 0.0f;
    }

    setRobotRearDistanceMM(meters * 1000.0f);
}