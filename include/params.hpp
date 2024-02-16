#pragma once

#include "Arduino.h"

/// DAC output to ESC
inline constexpr auto THROTTLE_PIN = PA4;
/// PWM output to steering servo
inline constexpr auto STEERING_PIN = PA7;
/// Encoder input pin
inline constexpr auto ENCODER_PIN = PA0;
/// Dac bit resolution
inline constexpr auto DAC_BITS = 12;
/// Dac resolution
inline constexpr auto DAC_RES = 1 << DAC_BITS;
/// Max steering angle of the kart, in degrees, symmetric
inline constexpr float MAX_STEERING = 24.0; // TODO
/// Time between encoder messages being sent
inline constexpr auto ENCODER_SAMPLE_PERIOD_US = 5 * 10000;
/// Teeth on the encoder rings
inline constexpr auto ENCODER_TEETH = 50.0f;
/// Wheel circumference in meters
inline constexpr float WHEEL_CIRC_METER = 2 * PI * (0.13335 / 2);