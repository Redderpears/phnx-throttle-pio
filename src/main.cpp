#include "ACAN_STM32.h"
#include "Arduino.h"
#include "candefs.hpp"
#include "utils.hpp"
#include "STM32TimerInterrupt.h"
#include "params.hpp"
#include <atomic>
#include <IWatchdog.h>

#define ENCODER_A_PIN PA0  // Must be interrupt-capable
#define ENCODER_B_PIN PA1

static void steering_rcv(const CANMessage &inMessage) {
    if (inMessage.id != CanID::SetAngle) {
        Serial.printf("Non steering msg in steering callback!\n");
        return;
    }

    float angle = inMessage.dataFloat[0];
    Serial.printf("Received message, setting steering to %f°\n", angle);

    float angle_as_bits = mapfloat(-angle, -MAX_STEERING, MAX_STEERING, 0, 4095.0);
    analogWrite(STEERING_PIN, uint16_t(angle_as_bits));
}

volatile std::atomic<bool> msg_to_send = false;
volatile CANMessage encoder_msg;
volatile std::atomic<int32_t> ticks = 0;
volatile std::atomic<int32_t> last_ticks = 0;

STM32Timer timer{TIM6};

static void timer_irq() {
    timer.detachInterrupt();

    int32_t current_count = ticks.load();
    int32_t ticks_delta = current_count - last_ticks;
    last_ticks = current_count;

    float ticks_per_ms = float(ticks_delta) / ENCODER_SAMPLE_PERIOD_US * 1000.0;
    float rps = ticks_per_ms * 1000.0f * (1.0f / ENCODER_TEETH);
    float meter_per_sec = rps * WHEEL_CIRC_METER; // (1/s * meter = meter/s)

    Serial.printf("Speed: %f m/s\n", meter_per_sec);

    encoder_msg.id = CanID::Encoder;
    encoder_msg.ext = true;
    encoder_msg.len = 6;
    encoder_msg.data16[0] = uint16_t(abs(ticks_delta));  // use abs for magnitude
    ((float *)(encoder_msg.data + 2))[0] = meter_per_sec;

    msg_to_send.store(true);

    if (!timer.attachInterruptInterval(ENCODER_SAMPLE_PERIOD_US, timer_irq)) {
        Serial.println("[TIMER] Failed to reattach interrupt.");
    }
}

static void encoderISR() {
    bool b_state = digitalRead(ENCODER_B_PIN);
    if (b_state) {
        ticks--;  // reverse direction
    } else {
        ticks++;  // forward direction
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(THROTTLE_PIN, OUTPUT);
    pinMode(STEERING_PIN, OUTPUT);
    
    pinMode(ENCODER_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER_B_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    delay(100);

    ACAN_STM32_Settings sett{500'000};
    ACAN_STM32::Filters filters;
    filters.addExtendedMask(CanID::SetAngle, CanID::SetAngle, ACAN_STM32::DATA, steering_rcv, ACAN_STM32::FIFO0);

    if (can.begin(sett, filters)) {
        Serial.println("[CAN] Initialized successfully.");
    } else {
        Serial.println("[CAN] Initialization failed!");
    }

    analogWriteResolution(DAC_BITS);

    if (timer.attachInterruptInterval(ENCODER_SAMPLE_PERIOD_US, timer_irq)) {
        Serial.printf("Timer started successfully with period: %u us\n", ENCODER_SAMPLE_PERIOD_US);
    } else {
        Serial.println("Timer init failed!");
    }

    attachInterrupt(ENCODER_A_PIN, encoderISR, RISING);

    analogWrite(STEERING_PIN, 4095.0 / 2);  // 50% steering

    if (IWatchdog.isReset(true)) {
        Serial.println("[WATCHDOG] Board reset via watchdog!");
        digitalWrite(LED_BUILTIN, HIGH);
    }

    IWatchdog.begin(100'000);  // 100 ms timeout
}

void loop() {
    IWatchdog.reload();
    can.dispatchReceivedMessage();

    noInterrupts();
    if (msg_to_send.load()) {
        can.tryToSendReturnStatus(const_cast<const CANMessage &>(encoder_msg));
        msg_to_send.store(false);
    }
    interrupts();
}
