#ifndef STEERING_CONTROL_HPP
#define STEERING_CONTROL_HPP

#include <Arduino.h>
#include <math.h>
#include "Config.hpp"

// Holds the laser-line angle at CFG.steering.target_angle_deg by producing a
// speed difference between the two tracks (left - right, m/s).
class SteeringControl {
public:
    void submitAngle(float angleDeg, uint32_t seq, uint32_t nowMs) {
        angleDeg_ = angleDeg;
        seq_ = seq;
        lastAngleMs_ = nowMs;
        everReceived_ = true;
        fresh_ = true;
    }

    // Marks angle data as invalid (compute module lost the line).
    void invalidate() {
        everReceived_ = false;
        fresh_ = false;
        integral_ = 0.0f;
    }

    bool hasAngle() const { return everReceived_; }
    float lastAngleDeg() const { return angleDeg_; }
    uint32_t lastSeq() const { return seq_; }
    uint32_t lastAngleAgeMs(uint32_t nowMs) const {
        return everReceived_ ? static_cast<uint32_t>(nowMs - lastAngleMs_) : 0xFFFFFFFFu;
    }

    bool isStale(uint32_t nowMs) const {
        return !everReceived_ || lastAngleAgeMs(nowMs) > CFG.steering.stale_ms;
    }

    float lastDiffMs() const { return lastDiffMs_; }

    void reset() {
        integral_ = 0.0f;
        lastDiffMs_ = 0.0f;
    }

    // Returns left-minus-right speed difference in m/s. meanSpeedMs is the
    // commanded mean track speed (signed, forward positive).
    float compute(float meanSpeedMs, uint32_t nowMs) {
        const SteeringCfg& c = CFG.steering;
        const uint32_t dtMs = static_cast<uint32_t>(nowMs - lastComputeMs_);
        lastComputeMs_ = nowMs;

        if (!c.enabled
            || meanSpeedMs < c.min_active_speed_ms
            || isStale(nowMs)) {
            integral_ = 0.0f;
            lastDiffMs_ = 0.0f;
            return 0.0f;
        }

        float errDeg = angleDeg_ - c.target_angle_deg;
        if (fabsf(errDeg) < c.deadband_deg) errDeg = 0.0f;
        const float errRad = errDeg * (PI / 180.0f);

        // Integrate only on fresh angles so a repeated sample is not counted twice.
        if (fresh_) {
            fresh_ = false;
            const float dt = (dtMs > 100) ? 0.1f : dtMs * 0.001f;
            integral_ += c.ki * errRad * dt;
            integral_ = constrain(integral_, -c.integral_limit_rad_s, c.integral_limit_rad_s);
        }

        const float yawRate = c.kp * errRad + integral_;         // rad/s
        float diff = static_cast<float>(c.sign) * yawRate * c.track_width_m;
        diff = constrain(diff, -c.max_diff_ms, c.max_diff_ms);

        lastDiffMs_ = diff;
        return diff;
    }

private:
    float angleDeg_ = 0.0f;
    uint32_t seq_ = 0;
    uint32_t lastAngleMs_ = 0;
    uint32_t lastComputeMs_ = 0;
    bool everReceived_ = false;
    bool fresh_ = false;
    float integral_ = 0.0f;
    float lastDiffMs_ = 0.0f;
};

#endif
