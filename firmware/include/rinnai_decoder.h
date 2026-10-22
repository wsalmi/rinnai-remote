#pragma once
#include <Arduino.h>

struct RinnaiStatus {
    bool valid;
    bool powerOn;
    bool flameActive;
    uint8_t temperature;
    uint8_t rawPower;
    uint8_t diagCode;
    uint8_t rawBytes[6];
};

class RinnaiDecoder {
public:
    static const uint32_t SYMBOL_PERIOD_MIN = 450;
    static const uint32_t SYMBOL_PERIOD_MAX = 800;
    static const uint32_t RESET_PULSE_MIN   = 900;
    static const uint32_t SHORT_PULSE_MAX   = 300;

    RinnaiDecoder(uint8_t rxPin) : _pin(rxPin), _packetReady(false), _bitIndex(0), _currByte(0) {}

    void begin() {
        pinMode(_pin, INPUT_PULLUP);
    }

    void handleInterrupt(uint32_t nowUs, bool pinState) {
        if (pinState == HIGH) {
            _lastRiseTime = nowUs;
        } else {
            uint32_t highDuration = nowUs - _lastRiseTime;
            uint32_t totalPeriod  = nowUs - _lastFallTime;
            _lastFallTime = nowUs;

            if (highDuration >= RESET_PULSE_MIN || totalPeriod >= RESET_PULSE_MIN) {
                _bitIndex = 0;
                _currByte = 0;
                return;
            }

            if (totalPeriod >= SYMBOL_PERIOD_MIN && totalPeriod <= SYMBOL_PERIOD_MAX) {
                uint8_t bit = (highDuration > SHORT_PULSE_MAX) ? 1 : 0;
                _currByte |= (bit << (_bitIndex % 8));
                _bitIndex++;

                if ((_bitIndex % 8) == 0) {
                    uint8_t bytePos = (_bitIndex / 8) - 1;
                    if (bytePos < 6) {
                        _rawPacket[bytePos] = _currByte;
                    }
                    _currByte = 0;

                    if (_bitIndex >= 48) {
                        _packetReady = true;
                        _bitIndex = 0;
                    }
                }
            }
        }
    }

    bool hasPacket() const { return _packetReady; }

    RinnaiStatus getStatus() {
        RinnaiStatus status;
        status.valid = false;

        if (!_packetReady) return status;

        uint8_t pkt[6];
        noInterrupts();
        memcpy(pkt, (const void*)_rawPacket, 6);
        _packetReady = false;
        interrupts();

        uint8_t calcXor = pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4];
        if (calcXor != pkt[5]) {
            return status;
        }

        memcpy(status.rawBytes, pkt, 6);
        status.valid = true;
        status.powerOn = (pkt[1] & 0x40) != 0;
        status.flameActive = (pkt[2] & 0x10) != 0;

        uint8_t tempCode = pkt[2] & 0x0F;
        if (tempCode == 0 && (pkt[1] & 0x08)) {
            status.temperature = 35;
        } else if (tempCode <= 11) {
            status.temperature = 37 + tempCode;
        } else if (tempCode == 12) {
            status.temperature = 50;
        } else if (tempCode == 13) {
            status.temperature = 55;
        } else if (tempCode == 14) {
            status.temperature = 60;
        } else {
            status.temperature = 37 + tempCode;
        }

        status.rawPower = pkt[3];
        status.diagCode = pkt[4];

        return status;
    }

private:
    uint8_t _pin;
    volatile bool _packetReady;
    volatile uint8_t _bitIndex;
    volatile uint8_t _currByte;
    volatile uint8_t _rawPacket[6];
    volatile uint32_t _lastFallTime = 0;
    volatile uint32_t _lastRiseTime = 0;
};
