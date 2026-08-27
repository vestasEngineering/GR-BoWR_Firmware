#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include "Config.hpp"

namespace FeedforwardConfiguration {

struct Values {
    float baseSpeedMs;
    float speedUpGain;
    float slowDownGain;
    float minimumTrackSpeedMs;
    float maximumTrackSpeedMs;
    float trackingDeadbandMm;
    float trackingAccelerationMps2;
    float trackingDecelerationMps2;
};

inline Values current() {
    return {
        CFG.ultrasonic.base_speed_ms,
        CFG.ultrasonic.kp_speed_up_track,
        CFG.ultrasonic.kp_slow_down_track,
        CFG.ultrasonic.min_track_speed_ms,
        CFG.ultrasonic.max_track_speed_ms,
        CFG.ultrasonic.tracking_deadband_mm,
        CFG.ultrasonic.track_accel_mps2,
        CFG.ultrasonic.track_decel_mps2,
    };
}

inline bool inRange(float value, float minimum, float maximum) {
    return isfinite(value) && value >= minimum && value <= maximum;
}

inline bool parseAndValidate(
    JsonArrayConst json,
    Values& out,
    const char*& error
) {
    static constexpr size_t
        expectedValueCount = 8;

    if (
        json.isNull() ||
        json.size() != expectedValueCount
    ) {
        error = "invalid_ff_value_count";
        return false;
    }

    for (
        size_t index = 0;
        index < expectedValueCount;
        ++index
    ) {
        JsonVariantConst value =
            json[index];

        if (
            !value.is<float>() &&
            !value.is<double>() &&
            !value.is<int>() &&
            !value.is<long>()
        ) {
            error =
                "non_numeric_ff_value";

            return false;
        }
    }

    out = {
        json[0].as<float>(),
        json[1].as<float>(),
        json[2].as<float>(),
        json[3].as<float>(),
        json[4].as<float>(),
        json[5].as<float>(),
        json[6].as<float>(),
        json[7].as<float>(),
    };

    if (
        !inRange(
            out.baseSpeedMs,
            0.0f,
            0.25f
        ) ||
        !inRange(
            out.speedUpGain,
            0.0f,
            0.02f
        ) ||
        !inRange(
            out.slowDownGain,
            0.0f,
            0.02f
        ) ||
        !inRange(
            out.minimumTrackSpeedMs,
            0.0f,
            0.25f
        ) ||
        !inRange(
            out.maximumTrackSpeedMs,
            0.0f,
            0.25f
        ) ||
        !inRange(
            out.trackingDeadbandMm,
            0.0f,
            50.0f
        ) ||
        !inRange(
            out.trackingAccelerationMps2,
            0.01f,
            5.0f
        ) ||
        !inRange(
            out.trackingDecelerationMps2,
            0.01f,
            10.0f
        )
    ) {
        error = "ff_value_out_of_range";
        return false;
    }

    if (
        out.minimumTrackSpeedMs
        > out.baseSpeedMs
    ) {
        error =
            "minimum_speed_exceeds_base";

        return false;
    }

    if (
        out.baseSpeedMs
        > out.maximumTrackSpeedMs
    ) {
        error =
            "base_speed_exceeds_maximum";

        return false;
    }

    return true;
}

inline void apply(const Values& values) {
    CFG.ultrasonic.base_speed_ms = values.baseSpeedMs;
    CFG.ultrasonic.kp_speed_up_track = values.speedUpGain;
    CFG.ultrasonic.kp_slow_down_track = values.slowDownGain;
    CFG.ultrasonic.min_track_speed_ms = values.minimumTrackSpeedMs;
    CFG.ultrasonic.max_track_speed_ms = values.maximumTrackSpeedMs;
    CFG.ultrasonic.tracking_deadband_mm = values.trackingDeadbandMm;
    CFG.ultrasonic.track_accel_mps2 = values.trackingAccelerationMps2;
    CFG.ultrasonic.track_decel_mps2 = values.trackingDecelerationMps2;
}

inline void writeJson(
    JsonArray target,
    const Values& values
) {
    target.add(
        values.baseSpeedMs
    );

    target.add(
        values.speedUpGain
    );

    target.add(
        values.slowDownGain
    );

    target.add(
        values.minimumTrackSpeedMs
    );

    target.add(
        values.maximumTrackSpeedMs
    );

    target.add(
        values.trackingDeadbandMm
    );

    target.add(
        values.trackingAccelerationMps2
    );

    target.add(
        values.trackingDecelerationMps2
    );
}

}  // namespace FeedforwardConfiguration
