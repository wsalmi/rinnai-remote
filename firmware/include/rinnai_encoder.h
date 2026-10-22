#pragma once
#include <Arduino.h>

enum class RinnaiButton {
    POWER_TOGGLE,
    TEMP_UP,
    TEMP_DOWN
};

class RinnaiEncoder {
public:
    static const uint32_t BIT_PERIOD_US   = 600;
    static const uint32_t SHORT_PULSE_US = 150;
    static const uint32_t LONG_PULSE_US  = 450;
    static const uint32_t RESET_PULSE_US = 1200;

    RinnaiEncoder(uint8_t txPin) : _pin(txPin) {}

    void begin() {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
    }

    void sendPacket(const uint8_t packet[6]) {
        // Pulso de Reset / Sync
        digitalWrite(_pin, HIGH);
        delayMicroseconds(RESET_PULSE_US);
        digitalWrite(_pin, LOW);
        delayMicroseconds(400);

        // Envia os 48 bits (6 bytes, LSB first)
        for (int b = 0; b < 6; b++) {
            uint8_t val = packet[b];
            for (int i = 0; i < 8; i++) {
                bool bit = (val >> i) & 1;
                uint32_t highTime = bit ? LONG_PULSE_US : SHORT_PULSE_US;
                uint32_t lowTime  = BIT_PERIOD_US - highTime;

                digitalWrite(_pin, HIGH);
                delayMicroseconds(highTime);
                digitalWrite(_pin, LOW);
                delayMicroseconds(lowTime);
            }
        }
        digitalWrite(_pin, LOW);
    }

    void pressButton(RinnaiButton btn) {
        uint8_t pkt[6] = {0x00, 0x00, 0x00, 0x00, 0xBF, 0x00};

        switch (btn) {
            case RinnaiButton::POWER_TOGGLE:
                pkt[1] = 0x01;
                break;
            case RinnaiButton::TEMP_UP:
                pkt[2] = 0x01;
                break;
            case RinnaiButton::TEMP_DOWN:
                pkt[2] = 0x02;
                break;
        }

        // Calcula paridade ímpar no MSB de cada byte
        for (int i = 0; i < 5; i++) {
            int ones = 0;
            for (int bit = 0; bit < 7; bit++) {
                if ((pkt[i] >> bit) & 1) ones++;
            }
            if ((ones % 2) == 0) {
                pkt[i] |= 0x80; // Adiciona paridade ímpar
            }
        }

        // Checksum XOR
        pkt[5] = pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4];

        // Transmite o comando
        sendPacket(pkt);
    }

private:
    uint8_t _pin;
};
