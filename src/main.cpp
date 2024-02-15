#include "ACAN_STM32.h"
#include "Arduino.h"
#include "candefs.hpp"
#include "utils.hpp"

// DAC output to ESC
constexpr auto THROTTLE_PIN = PA4;
// PWM output to steering servo
constexpr auto STEERING_PIN = PA7;
// Dac bit resolution
constexpr auto DAC_BITS = 12;
// Dac resolution
constexpr auto DAC_RES = 1 << DAC_BITS;
// Max steering angle of the kart, in degrees, symmetric
constexpr float MAX_STEERING = 24.0; // TODO

static void throttle_rcv(const CANMessage &inMessage) {
    // Blink the LED when we receive a CAN message
    digitalToggle(LED_BUILTIN);

    uint8_t percent = inMessage.data[0];

    Serial.printf("Received message, setting throttle to %hu %\n", percent);

    /*
    ESC uses differential voltage, so we need to invert our voltage.
    4096 is 3.1V, and the ESCs lowest value that will still move
    with no load is 3.8, so this should match roughly to the
    actual full range the ESC can be set to.
    */
    auto out_val = (uint16_t) (DAC_RES - (percent / 100.0) * DAC_RES);
    analogWrite(THROTTLE_PIN, out_val);
}

static void steering_rcv(const CANMessage &inMessage) {
    // Blink the LED when we receive a CAN message
    digitalToggle(LED_BUILTIN);

    // Left positive angle in degrees
    float angle = inMessage.dataFloat[0];

    Serial.printf("Received message, setting steering to %f %\n", angle);

    /*
     * We assume the steering motor firmware has configured our max angle as full/0 duty cycle, so we map the range of
     * our steering angles from positive to negative linearly, inverting because left is positive.
     */
    float angle_as_bits = mapfloat(-angle, -MAX_STEERING, MAX_STEERING, 0, DAC_RES);

    analogWrite(STEERING_PIN, uint16_t(angle_as_bits));
}

void setup() {
    // Pin setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(THROTTLE_PIN, OUTPUT);

    Serial.begin(9600);

    // Our bus is 512k baud
    ACAN_STM32_Settings sett{500'000};
    ACAN_STM32::Filters filters;

    // Throttle control messages
    filters.addExtendedMask(CanID::SetSpeed, CanID::SetSpeed, ACAN_STM32::DATA, throttle_rcv, ACAN_STM32::FIFO0);
    // Steering control messages
    filters.addExtendedMask(CanID::SetAngle, CanID::SetAngle, ACAN_STM32::DATA, steering_rcv, ACAN_STM32::FIFO0);

    if (can.begin(sett, filters)) {
        Serial.println("Can OK");
    }

    // Set our outputs to 12-bit resolution 0-4096
    analogWriteResolution(DAC_BITS);
}

void loop() {
    can.dispatchReceivedMessage();
}