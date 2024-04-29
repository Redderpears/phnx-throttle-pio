#include "ACAN_STM32.h"
#include "Arduino.h"
#include "candefs.hpp"
#include "utils.hpp"
#include "STM32TimerInterrupt.h"
#include "params.hpp"
#include <atomic>
#include <IWatchdog.h>

static void steering_rcv(const CANMessage &inMessage) {
    // Sanity check
    if (inMessage.id != CanID::SetAngle) {
        Serial.printf("Non steering msg in steering callback!\n");
        return;
    }

    // Left positive angle in degrees
    float angle = inMessage.dataFloat[0];

    Serial.printf("Received message, setting steering to %f %\n", angle);

    /*
     * We assume the steering motor firmware has configured our max angle as full/0 duty cycle, so we map the range of
     * our steering angles from positive to negative linearly, inverting because left is positive.
     */
    float angle_as_bits = mapfloat(-angle, -MAX_STEERING, MAX_STEERING, 0, 4092.0);

    analogWrite(STEERING_PIN, uint16_t(angle_as_bits));
}

volatile std::atomic<bool> msg_to_send = false;
volatile CANMessage encoder_msg;
volatile std::atomic<uint32_t> ticks = 0;
volatile uint32_t last_ticks = 0;

STM32Timer timer{TIM6};

static void timer_irq() {
    timer.detachInterrupt();
    uint32_t current_count = ticks.load();

    float ticks_per_ms = 0;
    uint32_t ticks_since_last_message = 0;

    // Find tick rate, handling overflow
    if (last_ticks > current_count) {
        uint32_t ticks_pre_overflow = std::numeric_limits<uint32_t>::max() - last_ticks;

        // Total ticks are ticks before we overflowed, and the ticks after
        ticks_since_last_message = ticks_pre_overflow + current_count;
        ticks_per_ms = float(ticks_since_last_message) / ENCODER_SAMPLE_PERIOD_US * 1000;
    } else {
        ticks_since_last_message = current_count - last_ticks;
        ticks_per_ms = float(ticks_since_last_message) / ENCODER_SAMPLE_PERIOD_US * 1000;
    }

    // ticks/sec * ms/s * rot/ticks = rot/sec
    float rps =
            ticks_per_ms * 1000.0f * (1.0f / ENCODER_TEETH);
    float rad_per_sec = rps * 2.0f * PI;

    float meter_per_sec = rad_per_sec * WHEEL_CIRC_METER;

    Serial.printf("Speed: %f m/s\n", meter_per_sec);

    // Setup can message for main loop
    encoder_msg.id = CanID::Encoder;
    encoder_msg.ext = true;
    encoder_msg.len = 6;
    encoder_msg.data16[0] = uint16_t(ticks_since_last_message);
    ((float *) (encoder_msg.data + 2))[0] = meter_per_sec;
    msg_to_send.store(true);

    last_ticks = current_count;
    timer.attachInterruptInterval(ENCODER_SAMPLE_PERIOD_US, timer_irq);
}

static void inc_count() {
    ticks.fetch_add(1);
}

void setup() {
    // Pin setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(STEERING_PIN, OUTPUT);
    pinMode(ENCODER_PIN, INPUT_PULLUP);

    Serial.begin(115200);

    // Our bus is 512k baud
    ACAN_STM32_Settings sett{500'000};
    ACAN_STM32::Filters filters;

    // Steering control messages
    filters.addExtendedMask(CanID::SetAngle, CanID::SetAngle, ACAN_STM32::DATA, steering_rcv, ACAN_STM32::FIFO0);

    if (can.begin(sett, filters)) {
        Serial.println("Can OK");
    }

    // Set our outputs to 12-bit resolution 0-4096
    analogWriteResolution(DAC_BITS);

    // Setup timer for encoders (5ms)
    if (timer.attachInterruptInterval(ENCODER_SAMPLE_PERIOD_US, timer_irq)) {
        Serial.printf("Timer started successfully with period: %u", ENCODER_SAMPLE_PERIOD_US);
    } else {
        Serial.println("Timer init failed!");
    }

    // Encoder interrupt
    attachInterrupt(ENCODER_PIN, inc_count, RISING);

    // Set steering to 50%
    analogWrite(STEERING_PIN, 4092.0 / 2);

    if (IWatchdog.isReset(true)) {
        Serial.println("Board reset via watchdog!");
        digitalWrite(LED_BUILTIN, HIGH);
    }

    // Reboot if main loop fails to cycle in 100ms (reboots board via hardware if it crashes)
    IWatchdog.begin(100'000);
}

void loop() {
    IWatchdog.reload();
    can.dispatchReceivedMessage();

    // Critical section over can message
    noInterrupts();
    if (msg_to_send.load()) {
        can.tryToSendReturnStatus(const_cast<const CANMessage &>(encoder_msg));
        msg_to_send.store(false);
    }
    interrupts();
}