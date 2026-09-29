#pragma once
/**
 * Version.hpp — single source of truth for robot + firmware identity.
 *
 * - Works on Arduino/Portenta H7 builds.
 * - Exposes compile-time constants and JSON helpers.
 * - Optional Git metadata via -D defines.
 */
#include <Arduino.h>

// ---------------------------
//  REQUIRED: Identify the robot
// ---------------------------
// Choose clear, short model IDs and a stable fleet ID (asset tag/SN).
#ifndef ROBOT_MODEL
  // e.g. "GR-LRR" for your unit; override via build flags for other models.
  #define ROBOT_MODEL        "GR-BoWR"
#endif

#ifndef ROBOT_FLEET_ID
  // e.g., field/production asset tag, or "proto-001"
  #define ROBOT_FLEET_ID     "001"
#endif

// ---------------------------
//  REQUIRED: Firmware semantic version
// ---------------------------
// Keep this updated when you make non-backward-compatible changes.
#ifndef FW_VERSION_MAJOR
  #define FW_VERSION_MAJOR   0
#endif
#ifndef FW_VERSION_MINOR
  #define FW_VERSION_MINOR   9
#endif
#ifndef FW_VERSION_PATCH
  #define FW_VERSION_PATCH   0
#endif

// Optional pre-release label: "alpha", "beta", "rc.1", empty for GA
#ifndef FW_VERSION_PRERELEASE
  #define FW_VERSION_PRERELEASE  "beta"
#endif

// ---------------------------
//  OPTIONAL: Build channel / environment
// ---------------------------
#ifndef BUILD_CHANNEL
  // e.g., "dev", "test", "prod"
  #define BUILD_CHANNEL       "dev"
#endif

// ---------------------------
//  OPTIONAL: Git metadata (inject via compiler -D flags or generator step)
// ---------------------------
#ifndef GIT_BRANCH
  #define GIT_BRANCH          ""
#endif
#ifndef GIT_COMMIT
  #define GIT_COMMIT          ""
#endif
#ifndef GIT_TAG
  #define GIT_TAG             ""
#endif
#ifndef GIT_DIRTY
  // "1" if there were uncommitted changes when built, else "0"
  #define GIT_DIRTY           "0"
#endif

// ---------------------------
//  Build date/time
// ---------------------------
// __DATE__ format: "Mmm dd yyyy"; __TIME__ format: "hh:mm:ss"
#ifndef BUILD_DATE
  #define BUILD_DATE          __DATE__
#endif
#ifndef BUILD_TIME
  #define BUILD_TIME          __TIME__
#endif

// ---------------------------
//  Board / Platform autodetect
// ---------------------------
#ifndef BUILD_BOARD
  #if defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4)
    #define BUILD_BOARD "Portenta H7"
  #elif defined(ARDUINO_ARCH_ESP32)
    #define BUILD_BOARD "ESP32"
  #elif defined(ARDUINO_ARCH_SAMD)
    #define BUILD_BOARD "SAMD"
  #elif defined(ARDUINO_ARCH_STM32)
    #define BUILD_BOARD "STM32"
  #else
    #define BUILD_BOARD "Arduino-Unknown"
  #endif
#endif

#ifndef BUILD_PLATFORM
  #if defined(ARDUINO)
    #define BUILD_PLATFORM "Arduino"
  #else
    #define BUILD_PLATFORM "Unknown"
  #endif
#endif

// ---------------------------
//  Feature flags (gate optional subsystems)
// ---------------------------
#ifndef FEATURE_ANDON
  #define FEATURE_ANDON 1
#endif
#ifndef FEATURE_ULTRASONIC
  #define FEATURE_ULTRASONIC 1
#endif
#ifndef FEATURE_ULTRASONIC_SERVO
  #define FEATURE_ULTRASONIC_SERVO 1
#endif
#ifndef FEATURE_ACTUATOR
  #define FEATURE_ACTUATOR 1
#endif
#ifndef FEATURE_BATTERY_OLED
  #define FEATURE_BATTERY_OLED 1
#endif
#ifndef FEATURE_MOTORS
  #define FEATURE_MOTORS 1
#endif
#ifndef ROBOT_HARDWARE_REVISION
  #define ROBOT_HARDWARE_REVISION "A"
#endif
#ifndef ROBOT_MAX_SUPPORTED_CHANNELS
  #define ROBOT_MAX_SUPPORTED_CHANNELS 8
#endif

// ---------------------------
//  Safety checks (required fields present)
// ---------------------------
static_assert(FW_VERSION_MAJOR >= 0, "FW_VERSION_MAJOR must be defined");
static_assert(FW_VERSION_MINOR >= 0, "FW_VERSION_MINOR must be defined");
static_assert(FW_VERSION_PATCH >= 0, "FW_VERSION_PATCH must be defined");

// ---------------------------
//  Compile-time helpers
// ---------------------------
namespace Version {

constexpr int  kMajor = FW_VERSION_MAJOR;
constexpr int  kMinor = FW_VERSION_MINOR;
constexpr int  kPatch = FW_VERSION_PATCH;

inline String semver() {
  String s = String(kMajor) + "." + String(kMinor) + "." + String(kPatch);
  const char* pre = FW_VERSION_PRERELEASE;
  if (pre && pre[0] != '\0') {
    s += "-";
    s += pre;
  }
  return s;
}

inline String firmwareId() {
  // e.g., "GR-LRR@0.9.0-beta"
  return String(ROBOT_MODEL) + "@" + semver();
}

inline String shortGit() {
  // prefer tag, else branch+short-sha
  const char* tag = GIT_TAG;
  const char* br  = GIT_BRANCH;
  const char* sha = GIT_COMMIT;
  bool dirty = (GIT_DIRTY[0] == '1');

  String s;
  if (tag && tag[0] != '\0') {
    s = tag;
  } else {
    if (br && br[0] != '\0') s += br;
    if (sha && sha[0] != '\0') {
      if (!s.isEmpty()) s += "@";
      // take first 7 chars if longer
      String shortSha = String(sha);
      if (shortSha.length() > 7) shortSha = shortSha.substring(0, 7);
      s += shortSha;
    }
  }
  if (dirty) s += "+dirty";
  return s;
}

inline String buildStamp() {
  return String(BUILD_DATE) + " " + String(BUILD_TIME);
}

inline String board()    { return String(BUILD_BOARD); }
inline String platform() { return String(BUILD_PLATFORM); }
inline String channel()  { return String(BUILD_CHANNEL); }
inline String model()    { return String(ROBOT_MODEL); }
inline String fleetId()  { return String(ROBOT_FLEET_ID); }
inline String hardwareRevision() {return String(ROBOT_HARDWARE_REVISION);}
inline int installedActuatorChannels() {return NUM_ACTUATORS;}
inline int maximumSupportedChannels() {return ROBOT_MAX_SUPPORTED_CHANNELS;}


inline void printHumanReadable(Stream& out = Serial) {
  out.println(F("===== Firmware Version ====="));
  out.print  (F("Robot Model: "));     out.println(model());
  out.print  (F("Fleet ID:    "));     out.println(fleetId());
  out.print  (F("Firmware:    "));     out.println(semver());
  out.print  (F("Build:       "));     out.println(buildStamp());
  out.print  (F("Board:       "));     out.println(board());
  out.print  (F("Platform:    "));     out.println(platform());
  out.print  (F("Channel:     "));     out.println(channel());
  out.print  (F("Git:         "));     out.println(shortGit());
  out.print  (F("Features:    "));
  out.print(F("ANDON_LIGHT=")); out.print(FEATURE_ANDON);
  out.print(F(", ULTRASONIC=")); out.print(FEATURE_ULTRASONIC);
  out.print(F(", ULTRASONIC_SERVO=")); out.print(FEATURE_ULTRASONIC_SERVO);
  out.print(F(", ACTUATOR=")); out.print(FEATURE_ACTUATOR);
  out.print(F(", BATTERY_OLED=")); out.print(FEATURE_BATTERY_OLED);
  out.print(F(", MOTORS=")); out.println(FEATURE_MOTORS);
  out.println(F("============================"));
}

inline void printJson(Stream& out = Serial) {
  out.print(F("{\"type\":\"fw_version\","));

  out.print(F("\"model\":\""));
  out.print(model());
  out.print(F("\","));

  out.print(F("\"fleet_id\":\""));
  out.print(fleetId());
  out.print(F("\","));

  out.print(F("\"hardware_revision\":\""));
  out.print(hardwareRevision());
  out.print(F("\","));

  out.print(F("\"semver\":\""));
  out.print(semver());
  out.print(F("\","));

  out.print(F("\"build\":\""));
  out.print(buildStamp());
  out.print(F("\","));

  out.print(F("\"board\":\""));
  out.print(board());
  out.print(F("\","));

  out.print(F("\"platform\":\""));
  out.print(platform());
  out.print(F("\","));

  out.print(F("\"channel\":\""));
  out.print(channel());
  out.print(F("\","));

  out.print(F("\"git\":\""));
  out.print(shortGit());
  out.print(F("\","));

  out.print(F("\"capabilities\":{"));

  out.print(F("\"max_supported_channels\":"));
  out.print(maximumSupportedChannels());
  out.print(F(","));

  out.print(F("\"atomic_trigger_update\":true,"));
  out.print(F("\"configuration_schema_version\":1"));

  out.print(F("},"));

  out.print(F("\"features\":{"));

  out.print(F("\"andon_light\":"));
  out.print(FEATURE_ANDON);
  out.print(F(","));

  out.print(F("\"ultrasonic\":"));
  out.print(FEATURE_ULTRASONIC);
  out.print(F(","));

  out.print(F("\"ultrasonic_servo\":"));
  out.print(FEATURE_ULTRASONIC_SERVO);
  out.print(F(","));

  out.print(F("\"actuator\":"));
  out.print(FEATURE_ACTUATOR);
  out.print(F(","));

  out.print(F("\"battery_oled\":"));
  out.print(FEATURE_BATTERY_OLED);
  out.print(F(","));

  out.print(F("\"motors\":"));
  out.print(FEATURE_MOTORS);

  out.print(F("}}"));
  out.println();
}

} // namespace Version