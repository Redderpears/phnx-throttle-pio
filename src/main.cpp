#include "ACAN_STM32.h"
#include "Arduino.h"

constexpr auto dac_pin = PA4;

static void can_rcv(const CANMessage &inMessage) {
    // Blink the LED when we receive a CAN message
    digitalToggle(LED_BUILTIN);

    uint8_t percent = inMessage.data[0];

    /*
    ESC uses differential voltage, so we need to invert our voltage.
    4092 is 3.1V, and the ESCs lowest value that will still move
    with no load is 3.8, so this should match roughly to the
    actual full range the ESC can be set to.
    */
    auto out_val = (uint16_t) (4092.0 - (percent / 100.0) * 4092.0);
    analogWrite(dac_pin, out_val);
}

void setup() {
    // Pin setup
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(dac_pin, OUTPUT);

    Serial.begin(9800);

    // Our bus is 512k baud
    ACAN_STM32_Settings sett{512'000};
    ACAN_STM32::Filters filters;
    // Only accept Set Speed messages
    filters.addExtendedMask(0x0000006, 0x0000006, ACAN_STM32::DATA, can_rcv, ACAN_STM32::FIFO0);

    if (can.begin(sett, filters)) {
        Serial.println("Can OK");
    }

    // Set our DAC to 12-bit resolution 0-4092
    analogWriteResolution(12);
}

void loop() {
    can.dispatchReceivedMessage();
}