#include "MotorCP.hpp"

MotorCP* MotorCP::instance_ = nullptr;

namespace {
inline int8_t signOf(float v) { return (v > 0.0f) ? 1 : ((v < 0.0f) ? -1 : 0); }
inline int8_t normDir(int8_t d) { return (d < 0) ? -1 : 1; }
}

MotorCP::MotorCP()
    : maxSpeedMs(CFG.motors.max_speed_ms),
      accelMps2(CFG.motors.accel_mps2),
      decelMps2(CFG.motors.decel_mps2),
      brakeDecelMps2(CFG.motors.brake_decel_mps2) {
    motorDirection[0] = CFG.motors.motor_direction[0];
    motorDirection[1] = CFG.motors.motor_direction[1];
    encoderDirection[0] = CFG.motors.encoder_direction[0];
    encoderDirection[1] = CFG.motors.encoder_direction[1];
}

void MotorCP::refreshDerived() {
    const MotorsCfg& c = CFG.motors;
    stepsPerMeter_ =
        static_cast<float>(c.steps_per_motor_rev) * c.gearbox_ratio
        / (PI * c.wheel_diameter_m);
    accelSpsPerMs_ = accelMps2 * stepsPerMeter_ * 0.001f;
    decelSpsPerMs_ = decelMps2 * stepsPerMeter_ * 0.001f;
    brakeSpsPerMs_ = brakeDecelMps2 * stepsPerMeter_ * 0.001f;
    maxSps_ = maxSpeedMs * stepsPerMeter_;
}

void MotorCP::setRampLimits(float maxMs, float accel, float decel, float brakeDecel) {
    if (maxMs > 0.0f) maxSpeedMs = maxMs;
    if (accel > 0.0f) accelMps2 = accel;
    if (decel > 0.0f) decelMps2 = decel;
    if (brakeDecel > 0.0f) brakeDecelMps2 = brakeDecel;
    refreshDerived();
}

void MotorCP::bindPins() {
    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];

        const PinName step = digitalPinToPinName(CFG.motors.pin_step[i]);
        const PinName dir  = digitalPinToPinName(CFG.motors.pin_dir[i]);
        const PinName en   = digitalPinToPinName(CFG.motors.pin_enable[i]);

        a.stepPort   = reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + STM_PORT(step) * 0x400UL);
        a.stepMask   = 1UL << STM_PIN(step);
        a.dirPort    = reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + STM_PORT(dir) * 0x400UL);
        a.dirMask    = 1UL << STM_PIN(dir);
        a.enablePort = reinterpret_cast<GPIO_TypeDef*>(GPIOA_BASE + STM_PORT(en) * 0x400UL);
        a.enableMask = 1UL << STM_PIN(en);
    }
}

void MotorCP::writeEnable(bool asserted) {
    const bool high = asserted ? CFG.motors.enable_active_high : !CFG.motors.enable_active_high;
    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];
        a.enablePort->BSRR = high ? a.enableMask : (a.enableMask << 16);
    }
}

void MotorCP::begin() {
    instance_ = this;
    refreshDerived();

    for (uint8_t i = 0; i < MOTORS; ++i) {
        pinMode(CFG.motors.pin_step[i], OUTPUT);
        pinMode(CFG.motors.pin_dir[i], OUTPUT);
        pinMode(CFG.motors.pin_enable[i], OUTPUT);
        pinMode(CFG.motors.pin_hlfb[i], INPUT);
        digitalWrite(CFG.motors.pin_step[i], LOW);
        digitalWrite(CFG.motors.pin_dir[i], LOW);
    }

    bindPins();
    motionInhibited_ = true;
    enabled_ = false;
    stepAllowed_ = false;
    writeEnable(false);

    resetEncoders();

    const pin_size_t irq0 = digitalPinToInterrupt(CFG.motors.pin_hlfb[0]);
    const pin_size_t irq1 = digitalPinToInterrupt(CFG.motors.pin_hlfb[1]);
    begun_ = (irq0 != NOT_AN_INTERRUPT) && (irq1 != NOT_AN_INTERRUPT);
    if (begun_) {
        attachInterrupt(irq0, hlfbIsr0, RISING);
        attachInterrupt(irq1, hlfbIsr1, RISING);
    }
}

void MotorCP::hlfbIsr0() { hlfbEdge(0); }
void MotorCP::hlfbIsr1() { hlfbEdge(1); }

void MotorCP::hlfbEdge(uint8_t axis) {
    MotorCP* m = instance_;
    if (!m) return;
    Axis& a = m->hw_[axis];

    const uint32_t now = micros();
    if (static_cast<uint32_t>(now - a.hlfbLastUs) < CFG.motors.hlfb_min_edge_us) return;
    a.hlfbLastUs = now;
    a.hlfbCount = a.hlfbCount + a.lastSign;
}

// ---------------------------------------------------------------------------
// Step generation (runs in interrupt context)
// ---------------------------------------------------------------------------

void MotorCP::stepIsr() {
    if (instance_) instance_->isrTick();
}

void MotorCP::rampAxes() {
    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];

        const float tgt = a.targetSps;
        const float cur = a.curSps;

        const bool speedingUp =
            (cur == 0.0f && tgt != 0.0f)
            || (cur * tgt > 0.0f && fabsf(tgt) > fabsf(cur));

        const float limit = speedingUp
            ? accelSpsPerMs_
            : (brake_ ? brakeSpsPerMs_ : decelSpsPerMs_);

        float dv = tgt - cur;
        if (dv > limit) dv = limit;
        else if (dv < -limit) dv = -limit;

        float nxt = cur + dv;
        // Always dwell at zero for one tick when reversing so DIR changes
        // only while no STEP pulses are being produced.
        if (cur != 0.0f && nxt * cur <= 0.0f) nxt = 0.0f;
        if (fabsf(nxt) < 1.0f) nxt = 0.0f;

        a.curSps = nxt;

        if (nxt == 0.0f) {
            a.phaseInc = 0;
            continue;
        }

        const int8_t sign = signOf(nxt);
        const int8_t level = (sign * a.motorDir > 0) ? 1 : -1;
        if (level != a.dirLatched) {
            a.dirPort->BSRR = (level > 0) ? a.dirMask : (a.dirMask << 16);
            a.dirLatched = level;
            a.dirChanged = true;
        }
        a.lastSign = sign;

        // 2^32 / ISR_HZ phase units per (step/s) per tick.
        float inc = fabsf(nxt) * (4294967296.0f / static_cast<float>(ISR_HZ));
        if (inc > 2147483648.0f) inc = 2147483648.0f;
        a.phaseInc = static_cast<uint32_t>(inc);
    }
}

void MotorCP::isrTick() {
    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];
        if (a.pulseTicks && --a.pulseTicks == 0) {
            a.stepPort->BSRR = a.stepMask << 16;
        }
    }

    if (++rampDiv_ >= (ISR_HZ / 1000)) {
        rampDiv_ = 0;
        rampAxes();
    }

    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];
        if (a.dirChanged) {
            a.dirChanged = false;
            continue;
        }
        const uint32_t inc = a.phaseInc;
        if (inc == 0) continue;

        const uint32_t next = a.phase + inc;
        const bool carry = next < a.phase;
        a.phase = next;

        if (carry && a.pulseTicks == 0) {
            a.stepPort->BSRR = a.stepMask;
            a.pulseTicks = CFG.motors.step_pulse_ticks ? CFG.motors.step_pulse_ticks : 1;
        }
    }
}

void MotorCP::hardZero() {
    noInterrupts();
    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];
        a.targetSps = 0.0f;
        a.curSps = 0.0f;
        a.phaseInc = 0;
    }
    interrupts();
}

// ---------------------------------------------------------------------------
// Main-context API
// ---------------------------------------------------------------------------

void MotorCP::setSpeeds(float s0, float s1, float, float) {
    speeds[0] = s0;
    speeds[1] = s1;
    if (s0 != 0.0f || s1 != 0.0f) brake_ = false;
}

void MotorCP::STOP() {
    speeds[0] = speeds[1] = 0.0f;
    hardZero();
}

void MotorCP::BRAKE_STOP() {
    speeds[0] = speeds[1] = 0.0f;
    brake_ = true;
    for (uint8_t i = 0; i < MOTORS; ++i) hw_[i].targetSps = 0.0f;
    steering.reset();
}

void MotorCP::setMotionInhibited(bool inhibited) {
    if (inhibited == motionInhibited_) return;
    motionInhibited_ = inhibited;
    if (inhibited) {
        speeds[0] = speeds[1] = 0.0f;
        hardZero();
        stepAllowed_ = false;
        enabled_ = false;
        writeEnable(false);
        steering.reset();
    }
}

void MotorCP::setMotorDirection(uint8_t axis, int8_t dir) {
    if (axis >= AXES) return;
    motorDirection[axis] = normDir(dir);
}

void MotorCP::setMotorDirections(int8_t d0, int8_t d1, int8_t, int8_t) {
    setMotorDirection(0, d0);
    setMotorDirection(1, d1);
}

void MotorCP::setEncoderDirection(uint8_t axis, int8_t dir) {
    if (axis >= AXES) return;
    encoderDirection[axis] = normDir(dir);
}

void MotorCP::setEncoderDirections(int8_t d0, int8_t d1, int8_t, int8_t) {
    setEncoderDirection(0, d0);
    setEncoderDirection(1, d1);
}

void MotorCP::applyTargets() {
    float v[MOTORS] = { speeds[0], speeds[1] };

    // Steering acts only on a straight, forward command (same speed on both
    // tracks), so single-axis tests and reversing are untouched.
    const int L = leftAxis();
    const int R = rightAxis();
    const float mean = 0.5f * (v[L] + v[R]);
    if (fabsf(v[L] - v[R]) < 0.001f && mean >= CFG.steering.min_active_speed_ms) {
        float diff = steering.compute(mean, millis());
        const float lim = 2.0f * mean;
        diff = constrain(diff, -lim, lim);
        v[L] = mean + 0.5f * diff;
        v[R] = mean - 0.5f * diff;
    } else {
        steering.reset();
    }

    for (uint8_t i = 0; i < MOTORS; ++i) {
        Axis& a = hw_[i];

        float sps = v[i] * stepsPerMeter_;
        sps = constrain(sps, -maxSps_, maxSps_);
        if (fabsf(v[i]) < ZERO_SPEED_THRESHOLD_MS) sps = 0.0f;

        if (!stepAllowed_ || motionInhibited_) sps = 0.0f;

        qpps[i] = static_cast<int32_t>(sps);
        if (a.curSps == 0.0f) a.motorDir = normDir(motorDirection[i]);
        a.targetSps = sps;
    }
}

void MotorCP::update() {
    const uint32_t now = millis();

    pollEncoders();

    if (motionInhibited_) {
        for (uint8_t i = 0; i < MOTORS; ++i) hw_[i].targetSps = 0.0f;
        return;
    }

    if (!enabled_) {
        writeEnable(true);
        enabled_ = true;
        settleStartMs_ = now;
        stepAllowed_ = false;
    } else if (!stepAllowed_
               && static_cast<uint32_t>(now - settleStartMs_) >= CFG.motors.enable_settle_ms) {
        stepAllowed_ = true;
    }

    applyTargets();
}

void MotorCP::resetEncoders() {
    noInterrupts();
    for (uint8_t i = 0; i < MOTORS; ++i) {
        hw_[i].hlfbCount = 0;
        hw_[i].hlfbLastUs = 0;
        lastCounts_[i] = 0;
        encCounts[i] = 0;
    }
    interrupts();
    lastSpeedMs_ = millis();
}

void MotorCP::pollEncoders() {
    for (uint8_t i = 0; i < MOTORS; ++i) {
        noInterrupts();
        const int32_t c = hw_[i].hlfbCount;
        interrupts();
        encCounts[i] = c;
    }

    const uint32_t now = millis();
    const uint32_t dtMs = static_cast<uint32_t>(now - lastSpeedMs_);
    if (dtMs >= 100) {
        for (uint8_t i = 0; i < MOTORS; ++i) {
            const int32_t delta =
                (encCounts[i] - lastCounts_[i]) * encoderDirection[i];
            lastCounts_[i] = encCounts[i];
            const float dt = dtMs * 0.001f;
            measuredPps_[i] = delta / dt;
            measuredMs_[i] = delta * CFG.motors.hlfb_mm_per_pulse * 0.001f / dt;
        }
        lastSpeedMs_ = now;
    }
}

int32_t MotorCP::getNormalizedCounts(uint8_t axis) const {
    if (axis >= MOTORS) return 0;
    return encCounts[axis] * encoderDirection[axis];
}

float MotorCP::getWheelMM(uint8_t axis) const {
    return getNormalizedCounts(axis) * CFG.motors.hlfb_mm_per_pulse;
}

float MotorCP::getRobotRearDistanceMM() const {
    return 0.5f * (getWheelMM(0) + getWheelMM(1));
}

float MotorCP::getRearWheelDisagreementMM() const {
    return fabsf(getWheelMM(0) - getWheelMM(1));
}

int32_t MotorCP::mm_to_distance_counts(float mm) const {
    return static_cast<int32_t>(lroundf(mm / CFG.motors.hlfb_mm_per_pulse));
}

void MotorCP::setRobotRearDistanceMM(float mm) {
    const int32_t counts = mm_to_distance_counts(mm);
    noInterrupts();
    for (uint8_t i = 0; i < MOTORS; ++i) {
        hw_[i].hlfbCount = counts * encoderDirection[i];
        encCounts[i] = hw_[i].hlfbCount;
        lastCounts_[i] = encCounts[i];
    }
    interrupts();
}

bool MotorCP::setRobotRearDistanceMMVerified(float mm, float toleranceMm) {
    setRobotRearDistanceMM(mm);
    lastEncoderRestoreWriteAttempts_ = 1;
    lastEncoderRestoreValidMask_ = encoderReadValidMask();
    lastEncoderRestoreMaximumErrorMM_ = fabsf(getRobotRearDistanceMM() - mm);
    // Counts are quantized to one HLFB pulse, so never demand better than that.
    const float limit = max(toleranceMm, CFG.motors.hlfb_mm_per_pulse);
    return begun_ && lastEncoderRestoreMaximumErrorMM_ <= limit;
}

float MotorCP::measuredSpeedMs(uint8_t axis) const {
    return (axis < MOTORS) ? measuredMs_[axis] : 0.0f;
}

bool MotorCP::readMotorMotionFeedback(
    uint8_t axis,
    int32_t& encoderCount,
    int32_t& speedQpps,
    uint8_t& encoderStatus,
    uint8_t& speedStatus
) {
    encoderStatus = 0;
    speedStatus = 0;
    if (axis >= MOTORS || !begun_) {
        encoderCount = 0;
        speedQpps = 0;
        return false;
    }
    pollEncoders();
    encoderCount = encCounts[axis];
    speedQpps = static_cast<int32_t>(measuredPps_[axis]);
    return true;
}
