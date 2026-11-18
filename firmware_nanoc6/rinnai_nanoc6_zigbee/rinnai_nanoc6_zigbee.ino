/**
 * Rinnai Smart Remote - M5NanoC6 (ESP32-C6) + Módulo HY-M154 (PC817 Tri-State)
 * Sketch standalone para Arduino IDE 2.x (Espressif esp32 >= 3.0.0).
 *
 * Configurações obrigatórias na Arduino IDE (Menu Tools):
 * - Board: M5NanoC6 (ou ESP32C6 Dev Module)
 * - USB CDC On Boot: Enabled
 * - Zigbee Mode: Zigbee ED (End Device)
 * - Partition Scheme: Zigbee 4MB with SPIFFS
 *
 * Pinagem Grove (HY2.0-4P):
 * - Preto    -> GND (Rinnai Fio 2)
 * - Vermelho -> 5V  (Rinnai Fio 1)
 * - Branco   -> G1  (TX Central Rinnai via divisor 10k/20k -> RX Passivo 2800 baud)
 * - Amarelo  -> G2  (Saída Tri-State -> HY-M154 modificado para '+' e '-')
 */

#ifndef ZIGBEE_MODE_ED
#error "Selecione Tools -> Zigbee Mode -> Zigbee ED (End Device) na Arduino IDE"
#endif

#include <Arduino.h>
#include "driver/gpio.h"
#include "Zigbee.h"
#include "esp_zigbee_core.h"

#define RINNAI_RX_PIN       1
#define RINNAI_TRISTATE_PIN 2
#define BLUE_LED_PIN        7
#define BTN_BOOT_PIN        9
#define RGB_PWR_PIN         19
#define RGB_DATA_PIN        20

#define ZIGBEE_ENDPOINT_ID  1
#define RESET_HOLD_MS       5000

// ============================================================================
// 1. PARSER & DECODIFICADOR UART PASSIVO (2800 baud)
// ============================================================================
class RinnaiFrameParser {
public:
    static constexpr size_t FRAME_LEN = 63;
    static constexpr uint8_t FLAME_BYTE = 0x77;
    static constexpr size_t FLAME_THRESHOLD = 10;

    static bool matchesPreamble(const uint8_t* data, size_t len) {
        if (len < 4) return false;
        return data[0] == 0x15 && data[1] == 0x45 && data[2] == 0x15 && data[3] == 0x51;
    }

    static size_t countFlameBytes(const uint8_t* data, size_t len) {
        size_t cnt = 0;
        for (size_t i = 0; i < len; i++) {
            if (data[i] == FLAME_BYTE) cnt++;
        }
        return cnt;
    }

    static bool checkPanelOff(const uint8_t f[FRAME_LEN]) {
        if (f[60] == 0x14 && f[62] == 0x31) return true;
        if (f[62] == 0x34 && f[60] == 0x14) return true;
        if (f[62] == 0x11 && f[60] == 0x45 && f[58] == 0x11) return true;
        if (f[62] == 0x0C && f[60] == 0x51 && f[58] == 0x11) return true;
        return false;
    }

    static size_t demodulateUartToBits(const uint8_t* bytes, size_t len, uint8_t* bitsOut, size_t maxBits) {
        if (len == 0 || len > 100) return 0;

        uint8_t rawBits[100 * 10];
        size_t rawLen = 0;
        for (size_t i = 0; i < len; i++) {
            uint8_t b = bytes[i];
            rawBits[rawLen++] = 0;
            for (uint8_t bit = 0; bit < 8; bit++) {
                rawBits[rawLen++] = (b >> bit) & 0x01;
            }
            rawBits[rawLen++] = 1;
        }

        size_t bitsLen = 0;
        uint8_t curr = rawBits[0];
        uint16_t cnt = 0;
        for (size_t i = 0; i < rawLen; i++) {
            if (rawBits[i] == curr) {
                cnt++;
            } else {
                if (curr == 0 && bitsLen < maxBits) {
                    bitsOut[bitsLen++] = (cnt >= 2) ? 1 : 0;
                }
                curr = rawBits[i];
                cnt = 1;
            }
        }
        if (curr == 0 && bitsLen < maxBits) {
            bitsOut[bitsLen++] = (cnt >= 2) ? 1 : 0;
        }
        return bitsLen;
    }

    static bool isTempBlockHeader(const uint8_t* p) {
        return (p[0] == 0x45 || p[0] == 0x44) &&
               (p[1] == 0x15 || p[1] == 0x14) &&
               (p[2] == 0x55 || p[2] == 0x54);
    }

    static int findTempBlockStart(const uint8_t* b, size_t len) {
        if (len < 49) return -1;
        size_t maxK = len - 3;
        if (maxK > 60) maxK = 60;
        for (size_t k = 46; k <= maxK; k++) {
            if (isTempBlockHeader(&b[k])) return (int)k;
        }
        return -1;
    }

    static int decodeTempFromBitsAndFrame(uint8_t p0, uint8_t p1, uint8_t units,
                                          const uint8_t* afterSyncBits, size_t afterSyncLen,
                                          const uint8_t* frame63OrNull) {
        if (p0 == 0 && p1 == 1) {
            if (units == 6) {
                if (frame63OrNull) {
                    uint8_t b62 = frame63OrNull[62];
                    uint8_t b61 = frame63OrNull[61];
                    uint8_t b60 = frame63OrNull[60];
                    uint8_t b58 = frame63OrNull[58];
                    uint8_t b59 = frame63OrNull[59];
                    if (b62 == 0x0D || b62 == 0x11 || b62 == 0x15 || b62 == 0x14 ||
                        b60 == 0x54 || b60 == 0x55 || b58 == 0x51 || b59 == 0x55) {
                        return 47;
                    }
                    if (b62 == 0xC4 || b62 == 0xC0 || b62 == 0x45 || b62 == 0x44 ||
                        b61 == 0x11 || b60 == 0x45) {
                        return 46;
                    }
                }
                if (afterSyncBits && afterSyncLen > 17) {
                    size_t firstOne = afterSyncLen;
                    for (size_t i = 17; i < afterSyncLen; i++) {
                        if (afterSyncBits[i] == 1) { firstOne = i; break; }
                    }
                    if (firstOne < afterSyncLen) {
                        size_t zerosAfter = 0;
                        for (size_t j = firstOne + 1; j < afterSyncLen && afterSyncBits[j] == 0; j++) {
                            zerosAfter++;
                        }
                        if (zerosAfter >= 5) return 47;
                        if (firstOne + 2 < afterSyncLen) return 46;
                    }
                }
                return frame63OrNull ? 47 : 46;
            } else if (units == 7) {
                return 47;
            }
            return 40 + units;
        } else if (p0 == 0 && p1 == 0) {
            if (units == 0) return 36;
            if (units == 1) return 37;
            if (units == 2) return 38;
            if (units == 3) {
                if (frame63OrNull) {
                    uint8_t b62 = frame63OrNull[62];
                    uint8_t b61 = frame63OrNull[61];
                    uint8_t b60 = frame63OrNull[60];
                    uint8_t b58 = frame63OrNull[58];
                    if ((b62 == 0x15 || b62 == 0x14 || b58 == 0x55 || b58 == 0x54 ||
                         b60 == 0x15 || b60 == 0x14) &&
                        b62 != 0x0C && b62 != 0x0D) {
                        return 35;
                    }
                    if (b62 == 0x0C || b62 == 0x0D || b62 == 0x44 || b62 == 0x45 ||
                        b61 == 0x51) {
                        return 39;
                    }
                }
                if (afterSyncBits && afterSyncLen > 17) {
                    size_t zeroRun = 0;
                    for (size_t i = 17; i < afterSyncLen && afterSyncBits[i] == 0; i++) {
                        zeroRun++;
                    }
                    if (zeroRun >= 23) return 35;
                    if (17 + zeroRun < afterSyncLen) return 39;
                }
                return 39;
            }
        } else if (p0 == 1 && p1 == 0) {
            if (units == 0) return (frame63OrNull && frame63OrNull[62] == 0x31) ? 56 : 48;
            if (units == 1) return (frame63OrNull && frame63OrNull[62] == 0x03) ? 57 : 49;
            if (units == 2) return (frame63OrNull && frame63OrNull[62] == 0x34) ? 58 : 50;
            if (units == 3) return (frame63OrNull && frame63OrNull[62] == 0x11) ? 59 : 51;
            if (units == 4) return (frame63OrNull && frame63OrNull[62] == 0x03) ? 60 : 52;
            if (units == 5) return 53;
            if (units == 6) return (frame63OrNull && frame63OrNull[62] == 0x14) ? 55 : 54;
            return 60;
        }
        return -1;
    }

    static int decodeTempFromPrefixUnits(uint8_t p0, uint8_t p1, uint8_t units, const uint8_t* frame63OrNull) {
        return decodeTempFromBitsAndFrame(p0, p1, units, nullptr, 0, frame63OrNull);
    }

    static int decodeTempFromSubpacket(const uint8_t* sub15, const uint8_t* frame63OrNull) {
        static const uint8_t syncPat[7] = {1, 0, 0, 0, 0, 0, 1};
        uint8_t tailBits[15 * 5];
        size_t tailBitsLen = demodulateUartToBits(sub15, 15, tailBits, sizeof(tailBits));
        for (size_t i = 0; i + 12 <= tailBitsLen; i++) {
            if (memcmp(&tailBits[i], syncPat, 7) == 0) {
                uint8_t p0 = tailBits[i + 7];
                uint8_t p1 = tailBits[i + 8];
                uint8_t units = (uint8_t)((tailBits[i + 9] << 2) | (tailBits[i + 10] << 1) | tailBits[i + 11]);
                int t = decodeTempFromBitsAndFrame(p0, p1, units, &tailBits[i], tailBitsLen - i, frame63OrNull);
                if (t > 0) return t;
                break;
            }
        }
        return -1;
    }

    static int calculateTemperature(const uint8_t frame[FRAME_LEN]) {
        static const uint8_t syncPat[7] = {1, 0, 0, 0, 0, 0, 1};

        if (isTempBlockHeader(frame + 48)) {
            int t = decodeTempFromSubpacket(frame + 48, frame);
            if (t > 0) return t;
        }

        uint8_t bits[FRAME_LEN * 5];
        size_t bitsLen = demodulateUartToBits(frame, FRAME_LEN, bits, sizeof(bits));

        int idx = -1;
        if (bitsLen >= 195 + 7) {
            for (size_t i = 195; i + 7 <= bitsLen; i++) {
                if (memcmp(&bits[i], syncPat, 7) == 0) { idx = (int)i; break; }
            }
        }
        if (idx == -1 && bitsLen >= 7) {
            for (int i = (int)bitsLen - 7; i >= 0; i--) {
                if (memcmp(&bits[i], syncPat, 7) == 0) { idx = i; break; }
            }
        }

        if (idx != -1 && (size_t)(idx + 7 + 5) <= bitsLen) {
            uint8_t p0 = bits[idx + 7];
            uint8_t p1 = bits[idx + 8];
            uint8_t units = (uint8_t)((bits[idx + 9] << 2) | (bits[idx + 10] << 1) | bits[idx + 11]);
            int t = decodeTempFromBitsAndFrame(p0, p1, units, &bits[idx], bitsLen - (size_t)idx, frame);
            if (t > 0) return t;
        }

        uint8_t lastByte = frame[62];
        uint8_t byte60   = frame[60];
        uint8_t byte61   = frame[61];

        if (lastByte == 0x15 ||
            (lastByte == 0x14 && (frame[51] == 0x54 || frame[51] == 0x14) &&
             (byte60 == 0x55 || byte60 == 0x35 || byte60 == 0x44))) {
            if (byte60 == 0x55) return 45;
            if (byte60 == 0x35) return 44;
            if (byte60 == 0x44) return (frame[58] == 0x54 && frame[51] == 0x14) ? 41 : 35;
            return 35;
        } else if (lastByte == 0xC4) {
            if (byte60 == 0x51) return 43;
            if (byte60 == 0x45) return 46;
            if (byte60 == 0x44) {
                if (frame[51] == 0x14 || frame[58] == 0x15 || frame[58] == 0x14 || frame[59] == 0x44) {
                    return 46;
                }
                return 36;
            }
        } else if (lastByte == 0x0D) {
            if (byte60 == 0x11) return (frame[51] == 0x14) ? 42 : 38;
            if (frame[51] == 0x14) return 47;
            return 37;
        } else if (lastByte == 0x0C) {
            return (byte61 == 0x14) ? 40 : 39;
        } else if (lastByte == 0x03) {
            if (byte61 == 0x44 && byte60 == 0x54) return 48;
            if (byte61 == 0x55) return 57;
            return 60;
        } else if (lastByte == 0x34) {
            return 58;
        } else if (lastByte == 0x11) {
            return 59;
        } else if (lastByte == 0x14) {
            return 55;
        }
        return -1;
    }

    static int calculateTemperatureFromBurst(const uint8_t* burst, size_t len) {
        if (len < 30 || len > 100) return -1;
        static const uint8_t syncPat[7] = {1, 0, 0, 0, 0, 0, 1};
        static const uint8_t zeroPad[12] = {0};

        size_t minK = (len > 24) ? (len - 24) : 0;
        for (size_t k = minK; k + 15 <= len; k++) {
            if (isTempBlockHeader(&burst[k])) {
                uint8_t tailFrame[FRAME_LEN] = {0};
                memcpy(tailFrame + 48, burst + k, 15);
                int t = decodeTempFromSubpacket(burst + k, tailFrame);
                if (t > 0) return t;
            }
        }

        uint8_t bits[100 * 5];
        size_t bitsLen = demodulateUartToBits(burst, len, bits, sizeof(bits));
        if (bitsLen < 40) return -1;

        for (size_t i = 0; i + 29 <= bitsLen; i++) {
            if (memcmp(&bits[i], syncPat, 7) == 0 &&
                bits[i + 12] == 0 && bits[i + 13] == 0 && bits[i + 14] == 0 && bits[i + 15] == 1) {
                bool hasZeroPad = false;
                size_t searchEnd = (i + 32 <= bitsLen) ? (i + 32) : bitsLen;
                for (size_t z = i + 16; z + 12 <= searchEnd; z++) {
                    if (memcmp(&bits[z], zeroPad, 12) == 0) { hasZeroPad = true; break; }
                }
                if (hasZeroPad) {
                    uint8_t p0 = bits[i + 7];
                    uint8_t p1 = bits[i + 8];
                    uint8_t units = (uint8_t)((bits[i + 9] << 2) | (bits[i + 10] << 1) | bits[i + 11]);
                    const uint8_t* f63 = (len == FRAME_LEN && matchesPreamble(burst, len)) ? burst : nullptr;
                    int t = decodeTempFromBitsAndFrame(p0, p1, units, &bits[i], bitsLen - i, f63);
                    if (t > 0) return t;
                }
            }
        }

        size_t startSearch = (bitsLen > 140) ? (bitsLen / 2) : 0;
        for (size_t i = startSearch; i + 28 <= bitsLen; i++) {
            if (memcmp(&bits[i], syncPat, 7) == 0 &&
                bits[i + 12] == 0 && bits[i + 13] == 0 && bits[i + 14] == 0) {
                bool hasZeroPad = false;
                size_t searchEnd = (i + 38 <= bitsLen) ? (i + 38) : bitsLen;
                for (size_t z = i + 15; z + 12 <= searchEnd; z++) {
                    if (memcmp(&bits[z], zeroPad, 12) == 0) { hasZeroPad = true; break; }
                }
                if (hasZeroPad) {
                    uint8_t p0 = bits[i + 7];
                    uint8_t p1 = bits[i + 8];
                    uint8_t units = (uint8_t)((bits[i + 9] << 2) | (bits[i + 10] << 1) | bits[i + 11]);
                    const uint8_t* f63 = (len == FRAME_LEN && matchesPreamble(burst, len)) ? burst : nullptr;
                    int t = decodeTempFromBitsAndFrame(p0, p1, units, &bits[i], bitsLen - i, f63);
                    if (t > 0) return t;
                }
            }
        }
        return -1;
    }
};

struct RinnaiTelemetryState {
    bool valid = false;
    bool powerOn = true;
    bool flameActive = false;
    uint8_t temperature = 0;
    uint32_t packetCount = 0;
    uint32_t tempPacketCount = 0;
    uint32_t rxByteCount = 0;
    uint8_t lastBurstSize = 0;
    uint32_t lastPacketMs = 0;
    uint8_t rawTail[12] = {0};
    uint8_t lastRawBytes[16] = {0};
    uint8_t lastRawLen = 0;

    bool isOnline(uint32_t nowMs) const {
        return valid && ((nowMs - lastPacketMs) < 5000);
    }
};

class RinnaiUartDecoder {
public:
    static constexpr size_t BUF_CAP = 256;
    static constexpr size_t BURST_CAP = 96;
    static constexpr uint32_t INTERFRAME_GAP_MS = 8;
    static constexpr uint32_t FLAME_HOLD_TIMEOUT_MS = 650;

    bool feedByte(uint8_t b, uint32_t nowMs) {
        bool updated = false;

        if (_burstLen > 0 && (nowMs - _lastByteMs) >= INTERFRAME_GAP_MS) {
            if (flushBurst(nowMs)) updated = true;
        }
        _lastByteMs = nowMs;

        _state.rxByteCount++;
        if (_state.lastRawLen < sizeof(_state.lastRawBytes)) {
            _state.lastRawBytes[_state.lastRawLen++] = b;
        } else {
            memmove(_state.lastRawBytes, _state.lastRawBytes + 1, sizeof(_state.lastRawBytes) - 1);
            _state.lastRawBytes[sizeof(_state.lastRawBytes) - 1] = b;
        }
        if (_burstLen < BURST_CAP) _burst[_burstLen++] = b;

        if (_winLen < RinnaiFrameParser::FRAME_LEN) {
            _win[_winLen++] = b;
            if (b == RinnaiFrameParser::FLAME_BYTE) _winFlameCount++;
        } else {
            if (_win[0] == RinnaiFrameParser::FLAME_BYTE && _winFlameCount > 0) _winFlameCount--;
            memmove(_win, _win + 1, RinnaiFrameParser::FRAME_LEN - 1);
            _win[RinnaiFrameParser::FRAME_LEN - 1] = b;
            if (b == RinnaiFrameParser::FLAME_BYTE) _winFlameCount++;
        }

        if (_winLen == RinnaiFrameParser::FRAME_LEN &&
            _winFlameCount >= RinnaiFrameParser::FLAME_THRESHOLD) {
            _lastFlameMs = nowMs;
            if (!_state.flameActive || (nowMs - _lastFlamePacketMs) >= 220) {
                _state.flameActive = true;
                _state.powerOn = true;
                _state.valid = true;
                _state.lastPacketMs = nowMs;
                _state.packetCount++;
                _lastFlamePacketMs = nowMs;
                updated = true;
            }
        }

        if (_bufLen >= BUF_CAP) {
            memmove(_buf, _buf + 128, _bufLen - 128);
            _bufLen -= 128;
        }
        _buf[_bufLen++] = b;

        while (_bufLen >= 4) {
            int idx = findPreambleFrom(0);
            if (idx < 0) {
                if (_bufLen > 3) {
                    memmove(_buf, _buf + (_bufLen - 3), 3);
                    _bufLen = 3;
                }
                break;
            }
            if (idx > 0) {
                memmove(_buf, _buf + idx, _bufLen - (size_t)idx);
                _bufLen -= (size_t)idx;
            }
            if (_bufLen < RinnaiFrameParser::FRAME_LEN) break;

            int nextPre = findPreambleFrom(4);
            int tempStart = RinnaiFrameParser::findTempBlockStart(_buf, _bufLen);
            size_t consumeLen = RinnaiFrameParser::FRAME_LEN;

            if (tempStart >= 46) {
                size_t neededLen = (size_t)tempStart + 15;
                if (nextPre >= 4 && (size_t)nextPre < neededLen) {
                    if (nextPre >= (int)RinnaiFrameParser::FRAME_LEN) {
                        processAlignedFrame(_buf, nowMs);
                        consumeLen = (size_t)nextPre;
                        updated = true;
                    } else {
                        consumeLen = (size_t)nextPre;
                    }
                } else {
                    if (_bufLen < neededLen) break;
                    uint8_t normFrame[RinnaiFrameParser::FRAME_LEN];
                    memcpy(normFrame, _buf, 48);
                    memcpy(normFrame + 48, _buf + tempStart, 15);
                    processAlignedFrame(normFrame, nowMs);
                    consumeLen = neededLen;
                    updated = true;
                }
            } else {
                if (nextPre >= 4 && nextPre < (int)RinnaiFrameParser::FRAME_LEN) {
                    consumeLen = (size_t)nextPre;
                } else {
                    processAlignedFrame(_buf, nowMs);
                    consumeLen = (nextPre == (int)RinnaiFrameParser::FRAME_LEN) ? (size_t)nextPre : RinnaiFrameParser::FRAME_LEN;
                    updated = true;
                }
            }

            memmove(_buf, _buf + consumeLen, _bufLen - consumeLen);
            _bufLen -= consumeLen;
        }
        return updated;
    }

    bool tick(uint32_t nowMs) {
        bool updated = false;
        if (_burstLen > 0 && (nowMs - _lastByteMs) >= INTERFRAME_GAP_MS) {
            if (flushBurst(nowMs)) updated = true;
        }
        if (_state.flameActive &&
            _winFlameCount < RinnaiFrameParser::FLAME_THRESHOLD &&
            (nowMs - _lastFlameMs) > FLAME_HOLD_TIMEOUT_MS) {
            _state.flameActive = false;
            updated = true;
        }
        return updated;
    }

    const RinnaiTelemetryState& state() const { return _state; }

private:
    int findPreambleFrom(size_t start) const {
        for (size_t i = start; i + 4 <= _bufLen; i++) {
            if (RinnaiFrameParser::matchesPreamble(&_buf[i], _bufLen - i)) return (int)i;
        }
        return -1;
    }

    bool flushBurst(uint32_t nowMs) {
        size_t len = _burstLen;
        _burstLen = 0;
        if (len < 30) return false;
        _state.lastBurstSize = (uint8_t)len;
        if (_lastAlignedMs != 0 && (nowMs - _lastAlignedMs) < 120) return false;

        if (RinnaiFrameParser::countFlameBytes(_burst, len) >= RinnaiFrameParser::FLAME_THRESHOLD) {
            _state.flameActive = true;
            _state.powerOn = true;
            _state.valid = true;
            _state.lastPacketMs = nowMs;
            _state.packetCount++;
            _lastFlameMs = nowMs;
            _lastFlamePacketMs = nowMs;
            return true;
        }

        int temp = RinnaiFrameParser::calculateTemperatureFromBurst(_burst, len);
        if (temp >= 35 && temp <= 60) {
            _state.flameActive = false;
            _state.powerOn = true;
            _state.temperature = (uint8_t)temp;
            _state.tempPacketCount++;
            _state.packetCount++;
            _state.lastPacketMs = nowMs;
            _state.valid = true;
            size_t tailCopy = (len >= 12) ? 12 : len;
            memcpy(_state.rawTail, _burst + (len - tailCopy), tailCopy);
            return true;
        }
        return false;
    }

    void processAlignedFrame(const uint8_t frame[RinnaiFrameParser::FRAME_LEN], uint32_t nowMs) {
        _lastAlignedMs = nowMs;
        _burstLen = 0;
        _state.packetCount++;
        _state.lastPacketMs = nowMs;
        _state.valid = true;
        memcpy(_state.rawTail, frame + (RinnaiFrameParser::FRAME_LEN - 12), 12);

        if (RinnaiFrameParser::countFlameBytes(frame, RinnaiFrameParser::FRAME_LEN) >= RinnaiFrameParser::FLAME_THRESHOLD) {
            _state.flameActive = true;
            _state.powerOn = true;
            _lastFlameMs = nowMs;
            _lastFlamePacketMs = nowMs;
            return;
        }

        _state.flameActive = false;
        _state.powerOn = !RinnaiFrameParser::checkPanelOff(frame);
        if (_state.powerOn) {
            int temp = RinnaiFrameParser::calculateTemperature(frame);
            if (temp >= 35 && temp <= 60) {
                _state.temperature = (uint8_t)temp;
                _state.tempPacketCount++;
            }
        }
    }

    uint8_t _buf[BUF_CAP] = {0};
    size_t _bufLen = 0;
    uint8_t _burst[BURST_CAP] = {0};
    size_t _burstLen = 0;
    uint32_t _lastByteMs = 0;
    uint32_t _lastAlignedMs = 0;
    uint8_t _win[RinnaiFrameParser::FRAME_LEN] = {0};
    size_t _winLen = 0;
    size_t _winFlameCount = 0;
    uint32_t _lastFlameMs = 0;
    uint32_t _lastFlamePacketMs = 0;
    RinnaiTelemetryState _state;
};

// ============================================================================
// 2. ATUADOR TRI-STATE (G2) & MALHA FECHADA COM TRAVA ANTI-LOOP
// ============================================================================
enum class RinnaiClickDir : uint8_t { NONE = 0, UP = 1, DOWN = 2 };

class Esp32TriStateHardware {
public:
    explicit Esp32TriStateHardware(uint8_t pin) : _pin(pin) {}
    void begin() { release(); }

    void release() {
        pinMode(_pin, INPUT);
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
    }

    void driveUp() {
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
        digitalWrite(_pin, HIGH);
        pinMode(_pin, OUTPUT);
    }

    void driveDown() {
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
        digitalWrite(_pin, LOW);
        pinMode(_pin, OUTPUT);
    }

private:
    uint8_t _pin;
};

class RinnaiClosedLoopController {
public:
    static constexpr uint8_t TEMP_MIN = 35;
    static constexpr uint8_t TEMP_MAX = 48;
    static constexpr uint32_t PRESS_DURATION_MS = 90;
    static constexpr uint32_t SETTLE_MIN_MS = 280;
    static constexpr uint32_t SETTLE_MAX_MS = 550;
    static constexpr uint8_t MAX_STALLED_CLICKS = 2;
    static constexpr uint8_t MAX_BLIND_CLICKS = 15;

    enum class State : uint8_t { IDLE, PRESSING, WAITING_SETTLE };
    using SetpointSyncCallback = void (*)(uint8_t syncedTemp, void* ctx);
    using ClickCallback = void (*)(RinnaiClickDir dir, void* ctx);

    explicit RinnaiClosedLoopController(Esp32TriStateHardware& hw) : _hw(hw) { _hw.release(); }

    void setCallbacks(SetpointSyncCallback onSync, ClickCallback onClick, void* ctx = nullptr) {
        _onSync = onSync;
        _onClick = onClick;
        _cbCtx = ctx;
    }

    static uint8_t clampTargetTemp(int tempC) {
        if (tempC < TEMP_MIN) return TEMP_MIN;
        if (tempC > TEMP_MAX) return TEMP_MAX;
        return (uint8_t)tempC;
    }

    void setTargetTemperature(int requestedTempC, const RinnaiTelemetryState& telemetry) {
        uint8_t clamped = clampTargetTemp(requestedTempC);
        if (!telemetry.valid || telemetry.temperature == 0) {
            _targetTemp = clamped;
            _hasPendingTarget = true;
            _stalledClicks = 0;
            return;
        }
        if (!telemetry.powerOn) {
            _hasPendingTarget = false;
            _stalledClicks = 0;
            _blindClicksRemaining = 0;
            _targetTemp = clampTargetTemp(telemetry.temperature);
            if (_onSync) _onSync(_targetTemp, _cbCtx);
            return;
        }

        _targetTemp = clamped;
        _stalledClicks = 0;
        if (!telemetry.flameActive && telemetry.temperature == _targetTemp) {
            _hasPendingTarget = false;
            _blindClicksRemaining = 0;
            return;
        }

        _hasPendingTarget = true;
        if (telemetry.flameActive) {
            int baseTemp = (_estimatedTemp >= 35 && _estimatedTemp <= 60) ? _estimatedTemp : telemetry.temperature;
            int diff = (int)_targetTemp - baseTemp;
            if (diff > MAX_BLIND_CLICKS) diff = MAX_BLIND_CLICKS;
            if (diff < -MAX_BLIND_CLICKS) diff = -MAX_BLIND_CLICKS;
            _blindClicksRemaining = (int8_t)diff;
            _reconcileAfterFlame = true;
            if (diff == 0) _hasPendingTarget = false;
        } else {
            _blindClicksRemaining = 0;
            _reconcileAfterFlame = false;
        }
    }

    void update(const RinnaiTelemetryState& telemetry, uint32_t nowMs) {
        if (telemetry.valid && telemetry.temperature > 0 && _lastObservedTemp == 0) {
            _lastObservedTemp = telemetry.temperature;
            _estimatedTemp = telemetry.temperature;
            if (!_hasPendingTarget) _targetTemp = clampTargetTemp(telemetry.temperature);
        }

        if (_wasFlameActive && !telemetry.flameActive && telemetry.valid && telemetry.temperature > 0) {
            _wasFlameActive = false;
            _blindClicksRemaining = 0;
            _estimatedTemp = telemetry.temperature;
            if (_reconcileAfterFlame) {
                _reconcileAfterFlame = false;
                _hasPendingTarget = false;
                _stalledClicks = 0;
                uint8_t realClamped = clampTargetTemp(telemetry.temperature);
                _targetTemp = realClamped;
                _lastObservedTemp = telemetry.temperature;
                if (_onSync) _onSync(realClamped, _cbCtx);
            }
        } else if (telemetry.flameActive) {
            _wasFlameActive = true;
        }

        if (_state == State::IDLE && !_hasPendingTarget &&
            telemetry.valid && !telemetry.flameActive && telemetry.powerOn &&
            telemetry.temperature > 0 && telemetry.temperature != _lastObservedTemp) {
            _lastObservedTemp = telemetry.temperature;
            _estimatedTemp = telemetry.temperature;
            uint8_t realClamped = clampTargetTemp(telemetry.temperature);
            _targetTemp = realClamped;
            if (_onSync) _onSync(realClamped, _cbCtx);
        }

        switch (_state) {
            case State::IDLE: {
                if (!_hasPendingTarget || !telemetry.valid || !telemetry.powerOn || telemetry.temperature == 0) return;
                RinnaiClickDir dir = RinnaiClickDir::NONE;
                if (telemetry.flameActive) {
                    if (_blindClicksRemaining > 0) {
                        dir = RinnaiClickDir::UP;
                        _blindClicksRemaining--;
                        if (_estimatedTemp < 60) _estimatedTemp++;
                    } else if (_blindClicksRemaining < 0) {
                        dir = RinnaiClickDir::DOWN;
                        _blindClicksRemaining++;
                        if (_estimatedTemp > 35) _estimatedTemp--;
                    } else {
                        _hasPendingTarget = false;
                        return;
                    }
                } else {
                    if (telemetry.temperature == _targetTemp) {
                        _hasPendingTarget = false;
                        _stalledClicks = 0;
                        _lastObservedTemp = telemetry.temperature;
                        return;
                    }
                    dir = (_targetTemp > telemetry.temperature) ? RinnaiClickDir::UP : RinnaiClickDir::DOWN;
                }

                _activeDir = dir;
                _tempBeforeClick = telemetry.temperature;
                _tempPktBeforeClick = telemetry.tempPacketCount;
                _stateStartMs = nowMs;
                _state = State::PRESSING;
                if (dir == RinnaiClickDir::UP) _hw.driveUp();
                else _hw.driveDown();
                if (_onClick) _onClick(dir, _cbCtx);
                break;
            }
            case State::PRESSING: {
                if ((nowMs - _stateStartMs) >= PRESS_DURATION_MS) {
                    _hw.release();
                    _stateStartMs = nowMs;
                    _state = State::WAITING_SETTLE;
                }
                break;
            }
            case State::WAITING_SETTLE: {
                uint32_t elapsed = nowMs - _stateStartMs;
                if (elapsed < SETTLE_MIN_MS) return;
                bool gotFreshTempPacket = (telemetry.tempPacketCount != _tempPktBeforeClick);
                if (!gotFreshTempPacket && elapsed < SETTLE_MAX_MS) return;

                if (!telemetry.flameActive || gotFreshTempPacket) {
                    _lastObservedTemp = telemetry.temperature;
                    _estimatedTemp = telemetry.temperature;
                    if (telemetry.temperature == _targetTemp) {
                        _hasPendingTarget = false;
                        _stalledClicks = 0;
                        _blindClicksRemaining = 0;
                        _reconcileAfterFlame = false;
                        _state = State::IDLE;
                        return;
                    }
                    bool movedCorrectly =
                        (_activeDir == RinnaiClickDir::UP && telemetry.temperature > _tempBeforeClick) ||
                        (_activeDir == RinnaiClickDir::DOWN && telemetry.temperature < _tempBeforeClick);
                    if (movedCorrectly) {
                        _stalledClicks = 0;
                    } else {
                        _stalledClicks++;
                        if (_stalledClicks >= MAX_STALLED_CLICKS) {
                            _hasPendingTarget = false;
                            _stalledClicks = 0;
                            _blindClicksRemaining = 0;
                            _reconcileAfterFlame = false;
                            uint8_t realClamped = clampTargetTemp(telemetry.temperature);
                            _targetTemp = realClamped;
                            if (_onSync) _onSync(realClamped, _cbCtx);
                            _state = State::IDLE;
                            return;
                        }
                    }
                } else if (_blindClicksRemaining == 0) {
                    _hasPendingTarget = false;
                }
                _state = State::IDLE;
                break;
            }
        }
    }

    uint8_t targetTemp() const { return _targetTemp; }

private:
    Esp32TriStateHardware& _hw;
    State _state = State::IDLE;
    uint32_t _stateStartMs = 0;
    RinnaiClickDir _activeDir = RinnaiClickDir::NONE;
    uint8_t _targetTemp = 40;
    uint8_t _tempBeforeClick = 0;
    uint8_t _lastObservedTemp = 0;
    uint8_t _estimatedTemp = 0;
    uint32_t _tempPktBeforeClick = 0;
    bool _hasPendingTarget = false;
    bool _wasFlameActive = false;
    bool _reconcileAfterFlame = false;
    uint8_t _stalledClicks = 0;
    int8_t _blindClicksRemaining = 0;
    SetpointSyncCallback _onSync = nullptr;
    ClickCallback _onClick = nullptr;
    void* _cbCtx = nullptr;
};

// ============================================================================
// 3. ENDPOINT ZIGBEE HA THERMOSTAT (Cluster 0x0201)
// ============================================================================
class RinnaiZigbeeThermostat : public ZigbeeEP {
public:
    explicit RinnaiZigbeeThermostat(uint8_t endpoint = 1) : ZigbeeEP(endpoint) {
        _device_id = ESP_ZB_HA_THERMOSTAT_DEVICE_ID;
        _cluster_list = esp_zb_zcl_cluster_list_create();

        esp_zb_basic_cluster_cfg_t basic_cfg = {
            .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE,
        };
        esp_zb_cluster_list_add_basic_cluster(
            _cluster_list, esp_zb_basic_cluster_create(&basic_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_identify_cluster_cfg_t identify_cfg = {
            .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,
        };
        esp_zb_cluster_list_add_identify_cluster(
            _cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_thermostat_cluster_cfg_t thermostat_cfg = {
            .local_temperature = 4000,
            .occupied_cooling_setpoint = 6000,
            .occupied_heating_setpoint = 4000,
            .control_sequence_of_operation = ESP_ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY,
            .system_mode = ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT,
        };
        esp_zb_attribute_list_t *thermostat_cluster = esp_zb_thermostat_cluster_create(&thermostat_cfg);
        struct LimitAttr { uint16_t id; int16_t *ptr; };
        const LimitAttr limits[] = {
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, &_absMinHeat},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, &_absMaxHeat},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,     &_minHeatLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,     &_maxHeatLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, &_absMinCool},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, &_absMaxCool},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID,     &_minCoolLimit},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID,     &_maxCoolLimit},
        };
        for (const auto &attr : limits) {
            esp_zb_thermostat_cluster_add_attr(thermostat_cluster, attr.id, attr.ptr);
        }
        esp_zb_cluster_add_attr(
            thermostat_cluster,
            ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT,
            ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID,
            ESP_ZB_ZCL_ATTR_TYPE_16BITMAP,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &_runningStateAttr
        );
        esp_zb_cluster_list_add_thermostat_cluster(_cluster_list, thermostat_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        _ep_config = {
            .endpoint = _endpoint,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = _device_id,
            .app_device_version = 0,
        };
    }

    void markStarted() {
        _stackReady = true;
        applySetpointLimits();
    }

    void applySetpointLimits() {
        if (!_stackReady || !Zigbee.started() || _limitsApplied) return;

        struct AttrInit { uint16_t id; int16_t val; };
        AttrInit attrs[] = {
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_HEAT_SETPOINT_LIMIT_ID, 3500},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_HEAT_SETPOINT_LIMIT_ID, 4800},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,     3500},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,     4800},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MIN_COOL_SETPOINT_LIMIT_ID, 5000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_ABS_MAX_COOL_SETPOINT_LIMIT_ID, 7000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MIN_COOL_SETPOINT_LIMIT_ID,     5000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_MAX_COOL_SETPOINT_LIMIT_ID,     7000},
            {ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,   6000},
        };

        esp_zb_lock_acquire(portMAX_DELAY);
        for (auto &entry : attrs) {
            esp_zb_zcl_set_attribute_val(
                _endpoint, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                entry.id, &entry.val, false);
        }
        esp_zb_lock_release();

        _limitsApplied = true;
        Serial.println("[ZIGBEE] Limites do Thermostat aplicados (Heat: 35-48°C, Cool: 60°C)");
    }

    bool consumePendingSetpoint(int &outTempC) {
        bool has = false;
        portENTER_CRITICAL(&_mux);
        if (_hasPendingSetpoint) { outTempC = _pendingSetpointC; _hasPendingSetpoint = false; has = true; }
        portEXIT_CRITICAL(&_mux);
        return has;
    }

    bool consumePendingModeRevert() {
        bool has = false;
        portENTER_CRITICAL(&_mux);
        if (_pendingModeRevert) { _pendingModeRevert = false; has = true; }
        portEXIT_CRITICAL(&_mux);
        return has;
    }

    void updateLocalTemperature(uint8_t tempC, bool forceReport = false) {
        if (tempC == 0) return;
        int16_t zclTemp = (int16_t)tempC * 100;
        if (!forceReport && zclTemp == _lastLocalTemp) return;
        _lastLocalTemp = zclTemp;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID, &zclTemp, true);
    }

    void updateHeatingSetpoint(uint8_t tempC, bool forceReport = false) {
        int16_t zclSetpoint = (int16_t)tempC * 100;
        portENTER_CRITICAL(&_mux);
        bool unchanged = (!forceReport && zclSetpoint == _lastSetpoint);
        if (!unchanged) _lastSetpoint = zclSetpoint;
        portEXIT_CRITICAL(&_mux);
        if (unchanged) return;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID, &zclSetpoint, true);
    }

    void updateSystemMode(bool powerOn, bool forceReport = false) {
        portENTER_CRITICAL(&_mux);
        _physicalPowerOn = powerOn;
        portEXIT_CRITICAL(&_mux);
        uint8_t mode = powerOn ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
        if (!forceReport && mode == _lastSystemMode) return;
        _lastSystemMode = mode;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID, &mode, true);
    }

    void updateRunningState(bool flameActive, bool forceReport = false) {
        uint16_t runState = flameActive ? 0x0001 : 0x0000;
        if (!forceReport && runState == _lastRunningState) return;
        _lastRunningState = runState;
        setAndReportAttr(ESP_ZB_ZCL_ATTR_THERMOSTAT_THERMOSTAT_RUNNING_STATE_ID, &runState, true);
    }

protected:
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override {
        if (!message || message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) return;
        if (message->info.cluster != ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT || !message->attribute.data.value) return;

        if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID &&
            message->attribute.data.size >= sizeof(int16_t)) {
            int16_t raw = *(const int16_t *)message->attribute.data.value;
            int tempC = (raw >= 0) ? ((raw + 50) / 100) : ((raw - 50) / 100);
            portENTER_CRITICAL(&_mux);
            _lastSetpoint = raw;
            _pendingSetpointC = tempC;
            _hasPendingSetpoint = true;
            portEXIT_CRITICAL(&_mux);
            Serial.printf("[ZIGBEE] Write occupied_heating_setpoint: %d°C (raw=%d)\n", tempC, raw);
        } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID &&
                   message->attribute.data.size >= sizeof(uint8_t)) {
            uint8_t reqMode = *(const uint8_t *)message->attribute.data.value;
            bool revert = false;
            portENTER_CRITICAL(&_mux);
            uint8_t expectedMode = _physicalPowerOn ? ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ESP_ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
            if (reqMode != expectedMode) { _pendingModeRevert = true; revert = true; }
            portEXIT_CRITICAL(&_mux);
            Serial.printf("[ZIGBEE] Write system_mode=%u (fisico=%u%s)\n",
                          reqMode, _physicalPowerOn ? 1 : 0, revert ? " -> revert" : "");
        }
    }

private:
    void setAndReportAttr(uint16_t attrId, void *valuePtr, bool sendReport) {
        if (!_stackReady || !Zigbee.started()) return;
        applySetpointLimits();
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(
            _endpoint, ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attrId, valuePtr, false);
        if (sendReport && Zigbee.connected()) {
            esp_zb_zcl_report_attr_cmd_t reportCmd = {};
            reportCmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
            reportCmd.attributeID = attrId;
            reportCmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
            reportCmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
            reportCmd.zcl_basic_cmd.src_endpoint = _endpoint;
            reportCmd.manuf_specific = 0;
            esp_zb_zcl_report_attr_cmd_req(&reportCmd);
        }
        esp_zb_lock_release();
    }

    int16_t _absMinHeat = 3500;
    int16_t _absMaxHeat = 4800;
    int16_t _minHeatLimit = 3500;
    int16_t _maxHeatLimit = 4800;
    int16_t _absMinCool = 5000;
    int16_t _absMaxCool = 7000;
    int16_t _minCoolLimit = 5000;
    int16_t _maxCoolLimit = 7000;
    uint16_t _runningStateAttr = 0x0000;
    int16_t _lastLocalTemp = -32768;
    int16_t _lastSetpoint = -32768;
    uint8_t _lastSystemMode = 0xFF;
    uint16_t _lastRunningState = 0xFFFF;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;
    bool _stackReady = false;
    bool _limitsApplied = false;
    volatile bool _physicalPowerOn = true;
    volatile bool _hasPendingSetpoint = false;
    volatile int _pendingSetpointC = 40;
    volatile bool _pendingModeRevert = false;
};

// ============================================================================
// 4. APLICAÇÃO PRINCIPAL (SETUP & LOOP)
// ============================================================================
HardwareSerial RinnaiSerial(1);
RinnaiUartDecoder decoder;
Esp32TriStateHardware triStateHw(RINNAI_TRISTATE_PIN);
RinnaiClosedLoopController controller(triStateHw);
RinnaiZigbeeThermostat zbThermostat(ZIGBEE_ENDPOINT_ID);

static uint32_t clickFlashUntilMs = 0;
static RinnaiClickDir clickFlashDir = RinnaiClickDir::NONE;
static uint32_t blueBlinkUntilMs = 0;

static constexpr uint32_t UART_PIN_PROBE_TIMEOUT_MS = 2500;
static uint8_t activeRxPin = RINNAI_TRISTATE_PIN;
static uint32_t lastPinSwitchMs = 0;
static uint32_t pktCountAtSwitch = 0;

static void onControllerSetpointSync(uint8_t syncedTemp, void *ctx) {
    (void)ctx;
    Serial.printf("[CTRL] Sincronizando setpoint para %u°C no Zigbee\n", syncedTemp);
    zbThermostat.updateHeatingSetpoint(syncedTemp, true);
}

static void onControllerClick(RinnaiClickDir dir, void *ctx) {
    (void)ctx;
    clickFlashDir = dir;
    clickFlashUntilMs = millis() + 140;
    Serial.printf("[ACT] Clique Tri-State: %s (G2=%s)\n",
                  dir == RinnaiClickDir::UP ? "TEMP +" : "TEMP -",
                  dir == RinnaiClickDir::UP ? "HIGH" : "LOW");
}

static void setBlueLed(bool on) {
    static int8_t lastBlue = -1;
    int8_t next = on ? 1 : 0;
    if (next != lastBlue) {
        lastBlue = next;
        digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
    }
}

static void setRgbColor(uint8_t r, uint8_t g, uint8_t b) {
    static int16_t lastR = -1;
    static int16_t lastG = -1;
    static int16_t lastB = -1;
    if (r != lastR || g != lastG || b != lastB) {
        lastR = r;
        lastG = g;
        lastB = b;
        rgbLedWrite(RGB_DATA_PIN, r, g, b);
    }
}

static void startPassiveUartOnPin(uint8_t newPin, uint32_t nowMs, bool isSwitch) {
    if (isSwitch) {
        RinnaiSerial.end();
        uint8_t prevPin = activeRxPin;
        if (prevPin == RINNAI_TRISTATE_PIN) {
            triStateHw.release();
        } else {
            pinMode(prevPin, INPUT);
            gpio_set_pull_mode((gpio_num_t)prevPin, GPIO_FLOATING);
        }
    }

    activeRxPin = newPin;
    lastPinSwitchMs = nowMs;
    pktCountAtSwitch = decoder.state().packetCount;

    RinnaiSerial.setRxBufferSize(256);
    RinnaiSerial.begin(2800, SERIAL_8N1, activeRxPin, -1);
    gpio_pulldown_dis((gpio_num_t)activeRxPin);
    gpio_pullup_en((gpio_num_t)activeRxPin);
    while (RinnaiSerial.available() > 0) {
        (void)RinnaiSerial.read();
    }
}

static void updateStatusLeds(uint32_t nowMs, bool zbConnected, bool btnPressed, uint32_t btnHoldMs,
                             const RinnaiTelemetryState &telemetry) {
    setBlueLed(nowMs < blueBlinkUntilMs);

    if (btnPressed && btnHoldMs > 400) {
        if (btnHoldMs >= RESET_HOLD_MS) {
            setRgbColor(80, 0, 0);
        } else {
            bool on = ((nowMs / 100) % 2) == 0;
            setRgbColor(on ? 50 : 0, 0, on ? 60 : 0);
        }
        return;
    }

    if (nowMs < clickFlashUntilMs) {
        if (clickFlashDir == RinnaiClickDir::UP) setRgbColor(0, 55, 55);
        else setRgbColor(55, 0, 55);
        return;
    }

    if (!zbConnected) {
        bool on = ((nowMs / 400) % 2) == 0;
        setRgbColor(0, 0, on ? 45 : 0);
        return;
    }

    if (telemetry.flameActive) {
        setRgbColor(65, 22, 0);
    } else if (telemetry.isOnline(nowMs) && telemetry.powerOn) {
        setRgbColor(0, 20, 0);
    } else if (telemetry.isOnline(nowMs) && !telemetry.powerOn) {
        setRgbColor(12, 12, 2);
    } else {
        setRgbColor(18, 12, 0);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n=== Rinnai Smart Remote - M5NanoC6 (Zigbee + HY-M154) ===");

    pinMode(BLUE_LED_PIN, OUTPUT);
    setBlueLed(false);
    pinMode(BTN_BOOT_PIN, INPUT_PULLUP);
    pinMode(RGB_PWR_PIN, OUTPUT);
    digitalWrite(RGB_PWR_PIN, HIGH);
    setRgbColor(0, 0, 30);

    triStateHw.begin();
    controller.setCallbacks(onControllerSetpointSync, onControllerClick, nullptr);

    startPassiveUartOnPin(RINNAI_TRISTATE_PIN, millis(), false);
    Serial.printf("[UART] Escuta passiva iniciada em G%u @ 2800 baud (8N1, pull-up ativo, auto-probe G2<->G1)\n",
                  activeRxPin);

    zbThermostat.setManufacturerAndModel("RinnaiCustom", "Rinnai-NanoC6");
    Zigbee.addEndpoint(&zbThermostat);

    bool eraseNvs = false;
    if (digitalRead(BTN_BOOT_PIN) == LOW) {
        uint32_t holdStart = millis();
        while (digitalRead(BTN_BOOT_PIN) == LOW) {
            if ((millis() - holdStart) >= 1500) {
                eraseNvs = true;
                setRgbColor(80, 0, 0);
                Serial.println("[ZIGBEE] Botao G9 pressionado no boot -> limpando NVRAM Zigbee!");
                break;
            }
            delay(20);
        }
    }

    Zigbee.setTimeout(8000);
    if (!Zigbee.begin(ZIGBEE_END_DEVICE, eraseNvs)) {
        if (!Zigbee.initialized()) {
            Serial.println("[ZIGBEE] Falha fatal ao inicializar stack! Reiniciando em 3s...");
            setRgbColor(80, 0, 0);
            delay(3000);
            ESP.restart();
        }
        Serial.println("[ZIGBEE] Coordenador sem resposta no boot; stack segue reconectando em background (segure G9 por 5s para novo pareamento).");
    }
    zbThermostat.markStarted();
}

void loop() {
    uint32_t nowMs = millis();

    bool telemetryUpdated = false;
    while (RinnaiSerial.available() > 0) {
        if (decoder.feedByte((uint8_t)RinnaiSerial.read(), nowMs)) {
            telemetryUpdated = true;
            blueBlinkUntilMs = nowMs + 35;
        }
    }
    if (decoder.tick(nowMs)) telemetryUpdated = true;

    const RinnaiTelemetryState &telemetry = decoder.state();

    bool pinHasRecentPacket = telemetry.valid &&
                                (telemetry.packetCount > pktCountAtSwitch) &&
                                ((nowMs - telemetry.lastPacketMs) < UART_PIN_PROBE_TIMEOUT_MS);
    if (!pinHasRecentPacket && (nowMs - lastPinSwitchMs) >= UART_PIN_PROBE_TIMEOUT_MS) {
        uint8_t nextPin = (activeRxPin == RINNAI_TRISTATE_PIN) ? RINNAI_RX_PIN : RINNAI_TRISTATE_PIN;
        Serial.printf("[UART] Sem pacotes validos em G%u apos %.1fs -> alternando para G%u\n",
                      activeRxPin, UART_PIN_PROBE_TIMEOUT_MS / 1000.0f, nextPin);
        startPassiveUartOnPin(nextPin, nowMs, true);
    }

    int requestedSetpoint = 0;
    if (zbThermostat.consumePendingSetpoint(requestedSetpoint)) {
        Serial.printf("[ZIGBEE->CTRL] Novo alvo solicitado: %d°C\n", requestedSetpoint);
        controller.setTargetTemperature(requestedSetpoint, telemetry);
        zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
    }
    if (zbThermostat.consumePendingModeRevert()) {
        Serial.printf("[ZIGBEE] Revertendo system_mode para estado fisico (%s)\n",
                      telemetry.powerOn ? "HEAT" : "OFF");
        zbThermostat.updateSystemMode(telemetry.powerOn, true);
    }

    if (activeRxPin != RINNAI_TRISTATE_PIN) {
        controller.update(telemetry, nowMs);
    }

    static bool wasZbConnected = false;
    static uint32_t lastPeriodicReportMs = 0;
    bool zbConnected = Zigbee.connected();
    bool justConnected = (zbConnected && !wasZbConnected);
    wasZbConnected = zbConnected;

    if (justConnected) {
        zbThermostat.applySetpointLimits();
        lastPeriodicReportMs = nowMs;
        if (telemetry.valid) {
            zbThermostat.updateSystemMode(telemetry.powerOn, true);
            zbThermostat.updateRunningState(telemetry.flameActive, true);
            if (telemetry.temperature > 0) {
                zbThermostat.updateLocalTemperature(telemetry.temperature, true);
                zbThermostat.updateHeatingSetpoint(
                    RinnaiClosedLoopController::clampTargetTemp(telemetry.temperature), true);
            }
        }
    } else if (zbConnected && telemetry.valid) {
        bool periodicSync = (nowMs - lastPeriodicReportMs) >= 60000;
        if (telemetryUpdated || periodicSync) {
            zbThermostat.updateSystemMode(telemetry.powerOn, periodicSync);
            zbThermostat.updateRunningState(telemetry.flameActive, periodicSync);
            if (telemetry.temperature > 0) zbThermostat.updateLocalTemperature(telemetry.temperature, periodicSync);
            if (periodicSync) {
                zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
                lastPeriodicReportMs = nowMs;
            }
        }
    }

    static bool wasPressed = false;
    static uint32_t pressStartMs = 0;
    bool pressed = (digitalRead(BTN_BOOT_PIN) == LOW);
    if (pressed && !wasPressed) pressStartMs = nowMs;
    else if (pressed && (nowMs - pressStartMs) >= RESET_HOLD_MS) {
        setRgbColor(80, 0, 0);
        delay(300);
        Zigbee.factoryReset(true);
    }
    wasPressed = pressed;
    updateStatusLeds(nowMs, zbConnected, pressed, pressed ? (nowMs - pressStartMs) : 0, telemetry);

    static uint32_t lastLogMs = 0;
    if ((nowMs - lastLogMs) >= 3000) {
        lastLogMs = nowMs;
        auto formatHex = [](const uint8_t *buf, size_t len, char *out, size_t outCap) {
            if (len == 0) {
                snprintf(out, outCap, "--");
                return;
            }
            size_t pos = 0;
            for (size_t i = 0; i < len && pos + 3 < outCap; i++) {
                pos += snprintf(out + pos, outCap - pos, "%s%02X", i == 0 ? "" : " ", buf[i]);
            }
        };
        char rawHexStr[52] = "--";
        char tailHexStr[40] = "--";
        formatHex(telemetry.lastRawBytes, telemetry.lastRawLen, rawHexStr, sizeof(rawHexStr));
        formatHex(telemetry.rawTail, telemetry.valid ? sizeof(telemetry.rawTail) : 0, tailHexStr, sizeof(tailHexStr));
        Serial.printf("[STATUS] ZB=%s | RxPin=G%u | Online=%s | Power=%s | Chama=%s | Temp=%u°C | Alvo=%u°C | Pkts=%u | RxBytes=%u | Burst=%u | G1=%d | G2=%d | TailHEX=[%s] | LastHEX=[%s]\n",
                      zbConnected ? "OK" : "PAIRING",
                      activeRxPin,
                      telemetry.isOnline(nowMs) ? "YES" : "NO",
                      telemetry.powerOn ? "HEAT" : "OFF",
                      telemetry.flameActive ? "ATIVA" : "IDLE",
                      telemetry.temperature,
                      controller.targetTemp(),
                      telemetry.packetCount,
                      telemetry.rxByteCount,
                      telemetry.lastBurstSize,
                      digitalRead(RINNAI_RX_PIN),
                      digitalRead(RINNAI_TRISTATE_PIN),
                      tailHexStr,
                      rawHexStr);
    }
    delay(5);
}
