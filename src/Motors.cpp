#include "Motors.hpp"
#include <math.h>

// ---------- Constructors ----------
Motors::Motors()
    : rcA(&roboclaw_uart_a, 10000),
      rcB(&roboclaw_uart_b, 10000) {}

Motors::Motors(UART* ua, UART* ub)
    : rcA(ua, 10000),
      rcB(ub, 10000) {}


void Motors::setMotorDirection(uint8_t axis, int8_t direction) {
    if (axis >= 4) return;
    motorDirection[axis] = direction < 0 ? -1 : 1;
}

void Motors::setMotorDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3) {
    setMotorDirection(0, d0);
    setMotorDirection(1, d1);
    setMotorDirection(2, d2);
    setMotorDirection(3, d3);
}

void Motors::setEncoderDirection(uint8_t axis, int8_t direction) {
    if (axis >= 4) return;
    encoderDirection[axis] = direction < 0 ? -1 : 1;
}

void Motors::setEncoderDirections(int8_t d0, int8_t d1, int8_t d2, int8_t d3) {
    setEncoderDirection(0, d0);
    setEncoderDirection(1, d1);
    setEncoderDirection(2, d2);
    setEncoderDirection(3, d3);
}

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
    // Sanitize all command inputs.
    if (!isfinite(s0)) s0 = 0.0f;
    if (!isfinite(s1)) s1 = 0.0f;
    if (!isfinite(s2)) s2 = 0.0f;
    if (!isfinite(s3)) s3 = 0.0f;

    noInterrupts();

    if (motionInhibited_) {
        speeds[0] = 0.0f;
        speeds[1] = 0.0f;
        speeds[2] = 0.0f;
        speeds[3] = 0.0f;

        interrupts();
        return;
    }

    speeds[0] = s0;
    speeds[1] = s1;
    speeds[2] = s2;
    speeds[3] = s3;

    if (fabsf(s0) >= ZERO_SPEED_THRESHOLD_MS ||
        fabsf(s1) >= ZERO_SPEED_THRESHOLD_MS ||
        fabsf(s2) >= ZERO_SPEED_THRESHOLD_MS ||
        fabsf(s3) >= ZERO_SPEED_THRESHOLD_MS) {
        closedLoopStopActive = false;
    }

    interrupts();
}

void Motors::setMotionInhibited(bool inhibited) {
    bool changed = false;

    noInterrupts();

    if (motionInhibited_ != inhibited) {
        motionInhibited_ = inhibited;
        changed = true;
    }

    // Always clear stored commands while inhibited.
    //
    // Also clear them when the inhibit is released so that an old command
    // cannot automatically resume motion.
    if (inhibited || changed) {
        speeds[0] = 0.0f;
        speeds[1] = 0.0f;
        speeds[2] = 0.0f;
        speeds[3] = 0.0f;
    }

    interrupts();

    if (changed && inhibited) {
        // Immediate command to the controllers.
        //
        // Use BRAKE_STOP() if controlled deceleration is mechanically safer.
        // Use STOP() if zero duty/coast is the intended fault response.
        BRAKE_STOP();
    }
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

    float command[4];
    bool inhibited;

    noInterrupts();

    inhibited = motionInhibited_;

    if (inhibited) {
        // Clear anything written directly to the public speeds array.
        speeds[0] = 0.0f;
        speeds[1] = 0.0f;
        speeds[2] = 0.0f;
        speeds[3] = 0.0f;
    }

    command[0] = speeds[0];
    command[1] = speeds[1];
    command[2] = speeds[2];
    command[3] = speeds[3];

    interrupts();

    if (inhibited) {
        // Continuously reinforce the stopped condition.
        rcA.SpeedAccelM1M2(ADDR_A, brakeDecelQppsPerSec, 0, 0);
        rcB.SpeedAccelM1M2(ADDR_B, brakeDecelQppsPerSec, 0, 0);
        return;
    }

    int32_t q0 = clamp_i32(
        ms_to_qpps(command[0]),
        -maxCommandQpps,
        maxCommandQpps
    );

    int32_t q1 = clamp_i32(
        ms_to_qpps(command[1]),
        -maxCommandQpps,
        maxCommandQpps
    );

    int32_t q2 = clamp_i32(
        ms_to_qpps(command[2]),
        -maxCommandQpps,
        maxCommandQpps
    );

    int32_t q3 = clamp_i32(
        ms_to_qpps(command[3]),
        -maxCommandQpps,
        maxCommandQpps
    );

    const uint32_t a_m1 = u32bits(q0 * motorDirection[0]);
    const uint32_t a_m2 = u32bits(q1 * motorDirection[1]);
    const uint32_t b_m1 = u32bits(q2 * motorDirection[2]);
    const uint32_t b_m2 = u32bits(q3 * motorDirection[3]);

    if (q0 == 0 && q1 == 0 && q2 == 0 && q3 == 0) {
        if (closedLoopStopActive) {
            rcA.SpeedAccelM1M2(
                ADDR_A,
                brakeDecelQppsPerSec,
                0,
                0
            );

            rcB.SpeedAccelM1M2(
                ADDR_B,
                brakeDecelQppsPerSec,
                0,
                0
            );
        } else {
            rcA.DutyM1M2(ADDR_A, 0, 0);
            rcB.DutyM1M2(ADDR_B, 0, 0);
        }

        return;
    }

    rcA.SpeedAccelM1M2(
        ADDR_A,
        accelQppsPerSec,
        a_m1,
        a_m2
    );

    rcB.SpeedAccelM1M2(
        ADDR_B,
        accelQppsPerSec,
        b_m1,
        b_m2
    );
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

bool Motors::readMotorMotionFeedback(
    uint8_t axis,
    int32_t& encoderCount,
    int32_t& speedQpps,
    uint8_t& encoderStatus,
    uint8_t& speedStatus
) {
    if (axis >= 4) {
        return false;
    }

    Basicmicro& controller = axis < 2 ? rcA : rcB;
    const uint8_t address = axis < 2 ? ADDR_A : ADDR_B;
    const bool channelM1 = (axis & 1u) == 0u;

    bool encoderValid = false;
    bool speedValid = false;
    uint8_t localEncoderStatus = 0;
    uint8_t localSpeedStatus = 0;

    const uint32_t encoderRaw = channelM1
        ? controller.ReadEncM1(
            address,
            &localEncoderStatus,
            &encoderValid
        )
        : controller.ReadEncM2(
            address,
            &localEncoderStatus,
            &encoderValid
        );

    const uint32_t speedRaw = channelM1
        ? controller.ReadSpeedM1(
            address,
            &localSpeedStatus,
            &speedValid
        )
        : controller.ReadSpeedM2(
            address,
            &localSpeedStatus,
            &speedValid
        );

    if (!encoderValid || !speedValid) {
        return false;
    }

    // Commit outputs only after both responses passed their CRC checks.
    encoderCount = static_cast<int32_t>(encoderRaw);
    speedQpps = static_cast<int32_t>(speedRaw);
    encoderStatus = localEncoderStatus;
    speedStatus = localSpeedStatus;

    // Keep the normal telemetry cache aligned with the direct diagnostic read.
    encCounts[axis] = encoderCount;
    qpps[axis] = speedQpps;
    encStatus_[axis] = encoderStatus;
    speedStatus_[axis] = speedStatus;
    encValid_[axis] = true;
    speedValid_[axis] = true;

    return true;
}

bool Motors::readControllerErrorFlags(
    uint8_t axis,
    uint32_t& errorFlags
) {
    if (axis >= 4) {
        return false;
    }

    Basicmicro& controller = axis < 2 ? rcA : rcB;
    const uint8_t address = axis < 2 ? ADDR_A : ADDR_B;

    bool valid = false;
    const uint32_t value = controller.ReadError(
        address,
        &valid
    );

    if (!valid) {
        return false;
    }

    errorFlags = value;
    return true;
}


uint8_t Motors::encoderReadValidMask() const {
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (encValid_[i]) mask |= static_cast<uint8_t>(1u << i);
    }
    return mask;
}

bool Motors::allEncoderReadsValid() const {
    return encoderReadValidMask() == 0x0F;
}

bool Motors::setRobotRearDistanceMMVerified(
    float mm,
    float toleranceMm,
    uint32_t timeoutMs,
    uint32_t retryDelayMs
) {
    if (
        !isfinite(mm) ||
        !isfinite(toleranceMm) ||
        toleranceMm < 0.0f
    ) {
        return false;
    }

    if (timeoutMs < 250) {
        timeoutMs = 250;
    }

    if (retryDelayMs < 20) {
        retryDelayMs = 20;
    }

    const int32_t normalizedTarget =
        mm_to_distance_counts(mm);

    const int32_t expectedRaw[4] = {
        normalizedTarget * encoderDirection[0],
        normalizedTarget * encoderDirection[1],
        normalizedTarget * encoderDirection[2],
        normalizedTarget * encoderDirection[3],
    };

    const int32_t toleranceCounts = max(
        static_cast<int32_t>(1),
        static_cast<int32_t>(
            ceilf(
                toleranceMm /
                DISTANCE_MM_PER_COUNT
            )
        )
    );

    lastEncoderRestoreValidMask_ = 0;
    lastEncoderRestoreWriteAttempts_ = 0;
    lastEncoderRestoreMaximumErrorMM_ = 0.0f;

    /*
     * Ensure no stored speed command can resume.
     *
     * Motion remains inhibited by EncoderSession throughout this
     * transaction. BRAKE_STOP additionally clears the stored speed
     * values and sends an immediate zero-speed command.
     */
    BRAKE_STOP();

    const uint32_t startedAtMs = millis();

    while (
        static_cast<uint32_t>(
            millis() - startedAtMs
        ) < timeoutMs
    ) {
        ++lastEncoderRestoreWriteAttempts_;

        /*
         * Write every encoder on every attempt.
         *
         * This is intentional. If the RoboClaws were still booting
         * during an earlier attempt, the entire logical position is
         * written again as one idempotent transaction.
         */
        rcA.SetEncM1(
            ADDR_A,
            u32bits(expectedRaw[0])
        );

        rcA.SetEncM2(
            ADDR_A,
            u32bits(expectedRaw[1])
        );

        rcB.SetEncM1(
            ADDR_B,
            u32bits(expectedRaw[2])
        );

        rcB.SetEncM2(
            ADDR_B,
            u32bits(expectedRaw[3])
        );

        /*
         * Give the controllers time to process the writes before
         * performing direct readback.
         */
        delay(40);

        int32_t actualRaw[4] = {
            0,
            0,
            0,
            0,
        };

        uint8_t validMask = 0;

        uint8_t status = 0;
        bool valid = false;

        actualRaw[0] = static_cast<int32_t>(
            rcA.ReadEncM1(
                ADDR_A,
                &status,
                &valid
            )
        );

        encStatus_[0] = status;
        encValid_[0] = valid;

        if (valid) {
            validMask |= 1u << 0;
        }

        status = 0;
        valid = false;

        actualRaw[1] = static_cast<int32_t>(
            rcA.ReadEncM2(
                ADDR_A,
                &status,
                &valid
            )
        );

        encStatus_[1] = status;
        encValid_[1] = valid;

        if (valid) {
            validMask |= 1u << 1;
        }

        status = 0;
        valid = false;

        actualRaw[2] = static_cast<int32_t>(
            rcB.ReadEncM1(
                ADDR_B,
                &status,
                &valid
            )
        );

        encStatus_[2] = status;
        encValid_[2] = valid;

        if (valid) {
            validMask |= 1u << 2;
        }

        status = 0;
        valid = false;

        actualRaw[3] = static_cast<int32_t>(
            rcB.ReadEncM2(
                ADDR_B,
                &status,
                &valid
            )
        );

        encStatus_[3] = status;
        encValid_[3] = valid;

        if (valid) {
            validMask |= 1u << 3;
        }

        lastEncoderRestoreValidMask_ =
            validMask;

        if (validMask == 0x0F) {
            bool allWithinTolerance = true;
            float maximumErrorMm = 0.0f;

            for (
                uint8_t axis = 0;
                axis < 4;
                ++axis
            ) {
                const int64_t difference =
                    static_cast<int64_t>(
                        actualRaw[axis]
                    ) -
                    static_cast<int64_t>(
                        expectedRaw[axis]
                    );

                const int64_t absoluteDifference =
                    difference < 0
                        ? -difference
                        : difference;

                const float errorMm =
                    static_cast<float>(
                        absoluteDifference
                    ) *
                    DISTANCE_MM_PER_COUNT;

                if (
                    errorMm >
                    maximumErrorMm
                ) {
                    maximumErrorMm =
                        errorMm;
                }

                if (
                    absoluteDifference >
                    toleranceCounts
                ) {
                    allWithinTolerance =
                        false;
                }
            }

            lastEncoderRestoreMaximumErrorMM_ =
                maximumErrorMm;

            if (allWithinTolerance) {
                /*
                 * Only update the local cache with values that were
                 * actually read back from the RoboClaws.
                 */
                for (
                    uint8_t axis = 0;
                    axis < 4;
                    ++axis
                ) {
                    encCounts[axis] =
                        actualRaw[axis];

                    qpps[axis] = 0;
                }

                closedLoopStopActive = true;

                return true;
            }
        }

        /*
         * Stay stopped while waiting for the next attempt.
         */
        BRAKE_STOP();

        delay(retryDelayMs);
    }

    /*
     * Do not update the local cache with the requested value after
     * failure. The session remains invalid and the CM5 retains the
     * durable last-valid checkpoint.
     */
    return false;
}

// ---------- Public loop ----------
void Motors::update() {
    sendSpeeds();
    pollEncoders();
}


// ---------- Safety ----------
void Motors::STOP() {
    noInterrupts();

    speeds[0] = 0.0f;
    speeds[1] = 0.0f;
    speeds[2] = 0.0f;
    speeds[3] = 0.0f;

    closedLoopStopActive = false;

    interrupts();

    rcA.DutyM1M2(ADDR_A, 0, 0);
    rcB.DutyM1M2(ADDR_B, 0, 0);
}

void Motors::BRAKE_STOP() {
    noInterrupts();

    speeds[0] = 0.0f;
    speeds[1] = 0.0f;
    speeds[2] = 0.0f;
    speeds[3] = 0.0f;

    closedLoopStopActive = true;

    interrupts();

    rcA.SpeedAccelM1M2(
        ADDR_A,
        brakeDecelQppsPerSec,
        0,
        0
    );

    rcB.SpeedAccelM1M2(
        ADDR_B,
        brakeDecelQppsPerSec,
        0,
        0
    );
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
    if (axis >= 4) return 0;
    return encCounts[axis] * encoderDirection[axis];
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

float Motors::getRearWheelDisagreementMM() const {
    return fabsf(getWheelMM(2) - getWheelMM(3));
}

int32_t Motors::mm_to_distance_counts(float mm) const {
    if (!isfinite(mm)) {
        mm = 0.0f;
    }

    return static_cast<int32_t>(lroundf(mm / DISTANCE_MM_PER_COUNT));
}

void Motors::setRobotRearDistanceMM(float mm) {
    if (!isfinite(mm)) mm = 0.0f;

    const int32_t normalizedCounts = mm_to_distance_counts(mm);
    const int32_t raw0 = normalizedCounts * encoderDirection[0];
    const int32_t raw1 = normalizedCounts * encoderDirection[1];
    const int32_t raw2 = normalizedCounts * encoderDirection[2];
    const int32_t raw3 = normalizedCounts * encoderDirection[3];

    STOP();
    delay(10);

    rcA.SetEncM1(ADDR_A, u32bits(raw0));
    rcA.SetEncM2(ADDR_A, u32bits(raw1));
    rcB.SetEncM1(ADDR_B, u32bits(raw2));
    rcB.SetEncM2(ADDR_B, u32bits(raw3));
    delay(20);

    encCounts[0] = raw0;
    encCounts[1] = raw1;
    encCounts[2] = raw2;
    encCounts[3] = raw3;
    for (int i = 0; i < 4; ++i) qpps[i] = 0;
}

void Motors::setRobotRearDistanceM(float meters) {
    if (!isfinite(meters)) {
        meters = 0.0f;
    }

    setRobotRearDistanceMM(meters * 1000.0f);
}

bool Motors::readMotorDiagnostic(
    uint8_t axis,
    MotorDiagnosticSnapshot& snapshot
) {
    if (axis >= 4) {
        return false;
    }

    Basicmicro& controller = axis < 2 ? rcA : rcB;
    const uint8_t address = axis < 2 ? ADDR_A : ADDR_B;
    const bool channelM1 = (axis & 1u) == 0u;

    MotorDiagnosticSnapshot sample{};
    bool valid = false;
    uint8_t status = 0;

    const uint32_t encoderRaw = channelM1
        ? controller.ReadEncM1(address, &status, &valid)
        : controller.ReadEncM2(address, &status, &valid);
    sample.encoderValid = valid;
    sample.encoderStatus = status;
    if (valid) sample.encoderCount = static_cast<int32_t>(encoderRaw);

    valid = false;
    status = 0;
    const uint32_t speedRaw = channelM1
        ? controller.ReadSpeedM1(address, &status, &valid)
        : controller.ReadSpeedM2(address, &status, &valid);
    sample.speedValid = valid;
    sample.speedStatus = status;
    if (valid) sample.speedQpps = static_cast<int32_t>(speedRaw);

    int16_t pwm1 = 0;
    int16_t pwm2 = 0;
    sample.pwmValid = controller.ReadPWMs(address, pwm1, pwm2);
    if (sample.pwmValid) sample.pwm = channelM1 ? pwm1 : pwm2;

    int16_t current1 = 0;
    int16_t current2 = 0;
    sample.currentValid = controller.ReadCurrents(address, current1, current2);
    if (sample.currentValid) sample.currentMa = channelM1 ? current1 : current2;

    valid = false;
    const uint32_t errors = controller.ReadError(address, &valid);
    sample.errorValid = valid;
    if (valid) sample.errorFlags = errors;

    // A useful diagnostic sample requires the four channel measurements.
    // Error validity is reported separately because older firmware may not
    // support every status command consistently.
    const bool requiredValid =
        sample.encoderValid &&
        sample.speedValid &&
        sample.pwmValid &&
        sample.currentValid;

    if (requiredValid) {
        snapshot = sample;
    }

    return requiredValid;
}

int8_t Motors::expectedEncoderDirection(uint8_t axis, float speedMps) const {
    if (axis >= 4 || !isfinite(speedMps) || speedMps == 0.0f) return 0;
    const int8_t robotDirection = speedMps > 0.0f ? 1 : -1;
    return static_cast<int8_t>(robotDirection * encoderDirection[axis]);
}
