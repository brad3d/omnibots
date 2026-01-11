/*
 * FrSky SBUS Helper
 * Maps SBUS channel values (172-1811, center 992) to various output ranges
 *
 * USAGE EXAMPLES WITH MANAGER (RECOMMENDED):
 *
 * // At top of main code, define manager and map channels:
 * SBUSManager rc;
 * SBUSChannel& leftArm = rc[0];     // Channel 0 = left arm
 * SBUSChannel& rightArm = rc[1];    // Channel 1 = right arm
 * SBUSChannel& jaw = rc[2];         // Channel 2 = jaw
 * SBUSChannel& enableSwitch = rc[5]; // Channel 5 = enable switch
 *
 * // In loop, update manager once:
 * if (sbus_rx.Read()) {
 *   data = sbus_rx.data();
 *   rc.update(data);  // Update all channels
 * }
 *
 * // Anywhere in your code, use friendly names:
 * leftArmMotor.setSpeed(leftArm.toMotorSpeed());   // -255 to 255
 * int jawPercent = jaw.toPercent();                // -100 to 100
 * bool enabled = enableSwitch.toSwitch();          // true/false
 *
 * DIRECT USAGE (without manager):
 *
 * // Map channel 0 to motor speed (-255 to 255)
 * int motorSpeed = FrSkySBUS::toMotorSpeed(data.ch[0]);
 *
 * // Map channel 1 to percentage (-100 to 100)
 * int percent = FrSkySBUS::toPercent(data.ch[1]);
 *
 * // Map channel 5 to on/off switch (>85% = on)
 * bool switchOn = FrSkySBUS::toSwitch(data.ch[5]);
 *
 * FrSky SBUS Range: 172 (min) to 1811 (max), center at 992
 */

#ifndef FRSKY_SBUS_H
#define FRSKY_SBUS_H

#include <Arduino.h>

class FrSkySBUS {
public:
  // SBUS channel range constants
  static const int SBUS_MIN = 172;      // -100% on transmitter
  static const int SBUS_CENTER = 992;   // 0% on transmitter
  static const int SBUS_MAX = 1811;     // +100% on transmitter

  // Deadband around center (prevents jitter near neutral)
  static const int DEADBAND = 10;

  /**
   * Map SBUS channel value to motor speed range (-255 to 255)
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return Motor speed (-255 to 255)
   */
  static int toMotorSpeed(int sbusValue) {
    return mapWithDeadband(sbusValue, -255, 255);
  }

  /**
   * Map SBUS channel value to percentage (-100 to 100)
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return Percentage (-100 to 100)
   */
  static int toPercent(int sbusValue) {
    return mapWithDeadband(sbusValue, -100, 100);
  }

  /**
   * Map SBUS channel value to range (-1000 to 1000)
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return Value (-1000 to 1000)
   */
  static int toThousands(int sbusValue) {
    return mapWithDeadband(sbusValue, -1000, 1000);
  }

  /**
   * Map SBUS channel value to normalized range (-1.0 to 1.0)
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return Normalized value (-1.0 to 1.0)
   */
  static float toNormalized(int sbusValue) {
    return mapWithDeadband(sbusValue, -1000, 1000) / 1000.0f;
  }

  /**
   * Map SBUS channel value to boolean (switch)
   * On if >= 85% of max position, off otherwise
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return true if on (>= 85%), false if off
   */
  static bool toSwitch(int sbusValue) {
    // 85% threshold: center + (max-center) * 0.85
    const long threshold = SBUS_CENTER + ((long)(SBUS_MAX - SBUS_CENTER) * 85 / 100);
    return sbusValue >= threshold;
  }

  /**
   * Map SBUS channel value to 3-position switch
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return -1 (down), 0 (center), 1 (up)
   */
  static int toThreeWaySwitch(int sbusValue) {
    const int lowerThreshold = SBUS_MIN + (SBUS_CENTER - SBUS_MIN) / 2;
    const int upperThreshold = SBUS_CENTER + (SBUS_MAX - SBUS_CENTER) / 2;

    if (sbusValue < lowerThreshold) return -1;
    if (sbusValue > upperThreshold) return 1;
    return 0;
  }

  /**
   * Check if channel is within deadband (near center)
   * @param sbusValue Raw SBUS channel value (172-1811)
   * @return true if within deadband
   */
  static bool isInDeadband(int sbusValue) {
    return abs(sbusValue - SBUS_CENTER) <= DEADBAND;
  }

private:
  /**
   * Map SBUS value to output range with center deadband
   * @param sbusValue Raw SBUS channel value
   * @param outMin Minimum output value
   * @param outMax Maximum output value
   * @return Mapped value with deadband applied
   */
  static int mapWithDeadband(int sbusValue, int outMin, int outMax) {
    // Apply deadband - return 0 if near center
    if (isInDeadband(sbusValue)) {
      return 0;
    }

    // Map to output range
    if (sbusValue < SBUS_CENTER) {
      // Map lower half: SBUS_MIN to SBUS_CENTER -> outMin to 0
      return map(sbusValue, SBUS_MIN, SBUS_CENTER - DEADBAND, outMin, 0);
    } else {
      // Map upper half: SBUS_CENTER to SBUS_MAX -> 0 to outMax
      return map(sbusValue, SBUS_CENTER + DEADBAND, SBUS_MAX, 0, outMax);
    }
  }
};

// ============================================================================
// SBUS CHANNEL CLASS
// ============================================================================

/**
 * Represents a single SBUS channel with conversion methods
 * Stores the current value and provides easy access to mapped ranges
 */
class SBUSChannel {
private:
  int value;

public:
  SBUSChannel() : value(FrSkySBUS::SBUS_CENTER) {}

  /**
   * Update the channel value
   * @param sbusValue Raw SBUS value (172-1811)
   */
  void update(int sbusValue) {
    value = sbusValue;
  }

  /**
   * Get raw SBUS value
   * @return Raw value (172-1811)
   */
  int raw() const {
    return value;
  }

  /**
   * Convert to motor speed (-255 to 255)
   */
  int toMotorSpeed() const {
    return FrSkySBUS::toMotorSpeed(value);
  }

  /**
   * Convert to percentage (-100 to 100)
   */
  int toPercent() const {
    return FrSkySBUS::toPercent(value);
  }

  /**
   * Convert to thousands (-1000 to 1000)
   */
  int toThousands() const {
    return FrSkySBUS::toThousands(value);
  }

  /**
   * Convert to normalized float (-1.0 to 1.0)
   */
  float toNormalized() const {
    return FrSkySBUS::toNormalized(value);
  }

  /**
   * Convert to switch (true if >= 85%)
   */
  bool toSwitch() const {
    return FrSkySBUS::toSwitch(value);
  }

  /**
   * Convert to 3-way switch (-1, 0, 1)
   */
  int toThreeWaySwitch() const {
    return FrSkySBUS::toThreeWaySwitch(value);
  }

  /**
   * Check if in deadband (near center)
   */
  bool isInDeadband() const {
    return FrSkySBUS::isInDeadband(value);
  }
};

// ============================================================================
// SBUS MANAGER CLASS
// ============================================================================

/**
 * Manages all SBUS channels and provides easy access via friendly names
 *
 * Usage:
 *   SBUSManager rc;
 *   SBUSChannel& leftArm = rc[0];
 *
 *   rc.update(data);
 *   int speed = leftArm.toMotorSpeed();
 */
class SBUSManager {
private:
  SBUSChannel channels[16];  // SBUS has 16 channels

public:
  /**
   * Access a channel by index
   * @param index Channel number (0-15)
   * @return Reference to the channel
   */
  SBUSChannel& operator[](int index) {
    if (index < 0 || index >= 16) {
      // Return first channel as fallback (prevent crash)
      return channels[0];
    }
    return channels[index];
  }

  /**
   * Update all channels from SBUS data
   * Call this once per loop after reading SBUS
   * @param data SBUS data frame from sbus_rx.data()
   */
  void update(const bfs::SbusData& data) {
    for (int i = 0; i < data.NUM_CH; i++) {
      channels[i].update(data.ch[i]);
    }
  }

  /**
   * Get number of channels
   */
  int getChannelCount() const {
    return 16;
  }
};

#endif // FRSKY_SBUS_H
