#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief Analisador puro dos quadros UART de 2800 baud (63 bytes) do barramento Rinnai.
 *
 * Porta diretamente a lógica calibrada em hardware de tools/live_dashboard.py:
 * - Alinhamento pelo preâmbulo 0x15 0x45 0x15 0x51 (63 bytes)
 * - Fallback por silêncio entre rajadas (>= 8 ms) e marcador de bits '1000001'
 * - Detecção de chama ativa por inversão de polaridade (contagem de 0x77 >= 10)
 * - Detecção de estado Power ON / Standby (checkPanelOff)
 * - Decodificação analítica de temperatura (35°C a 60°C) com fallback pelo trailer
 */
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
        // Faixa superior (48..60°C): trailer termina em 0x31 ou 0x34 com b[60] == 0x14
        if (f[60] == 0x14 && f[62] == 0x31) return true;
        if (f[62] == 0x34 && f[60] == 0x14) return true;
        // Faixa 47°C: termina em 0x11 com b[60] == 0x45 e b[58] == 0x11
        if (f[62] == 0x11 && f[60] == 0x45 && f[58] == 0x11) return true;
        // Faixa 42°C: termina em 0x0C com b[60] == 0x51 e b[58] == 0x11
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
            if (isTempBlockHeader(&b[k])) {
                return (int)k;
            }
        }
        return -1;
    }

    static int decodeTempFromBitsAndFrame(uint8_t p0, uint8_t p1, uint8_t units,
                                          const uint8_t* afterSyncBits, size_t afterSyncLen,
                                          const uint8_t* frame63OrNull) {
        if (p0 == 0 && p1 == 1) {
            if (units == 6) {
                // 46°C vs 47°C
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
                        if (afterSyncBits[i] == 1) {
                            firstOne = i;
                            break;
                        }
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
                // 35°C (Modo Verão) vs 39°C
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

        // 1. Tenta primeiro diretamente no bloco normalizado de temperatura (bytes 48..62)
        if (isTempBlockHeader(frame + 48)) {
            int t = decodeTempFromSubpacket(frame + 48, frame);
            if (t > 0) return t;
        }

        // 2. Busca no quadro completo de 63 bytes
        uint8_t bits[FRAME_LEN * 5];
        size_t bitsLen = demodulateUartToBits(frame, FRAME_LEN, bits, sizeof(bits));

        int idx = -1;
        if (bitsLen >= 195 + 7) {
            for (size_t i = 195; i + 7 <= bitsLen; i++) {
                if (memcmp(&bits[i], syncPat, 7) == 0) {
                    idx = (int)i;
                    break;
                }
            }
        }
        if (idx == -1 && bitsLen >= 7) {
            for (int i = (int)bitsLen - 7; i >= 0; i--) {
                if (memcmp(&bits[i], syncPat, 7) == 0) {
                    idx = i;
                    break;
                }
            }
        }

        if (idx != -1 && (size_t)(idx + 7 + 5) <= bitsLen) {
            uint8_t p0 = bits[idx + 7];
            uint8_t p1 = bits[idx + 8];
            uint8_t units = (uint8_t)((bits[idx + 9] << 2) | (bits[idx + 10] << 1) | bits[idx + 11]);
            int t = decodeTempFromBitsAndFrame(p0, p1, units, &bits[idx], bitsLen - (size_t)idx, frame);
            if (t > 0) return t;
        }

        // 3. Discriminador secundário via bytes finais (caso haja ruído no trem de pulsos)
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

    /**
     * @brief Fallback tolerante a deslocamento de fase UART em uma rajada separada por silêncio.
     * Procura o padrão: '1000001' + p0 p1 u2 u1 u0 + '000' seguido de >= 12 zeros em sequência.
     */
    static int calculateTemperatureFromBurst(const uint8_t* burst, size_t len) {
        if (len < 30 || len > 100) return -1;
        static const uint8_t syncPat[7] = {1, 0, 0, 0, 0, 0, 1};
        static const uint8_t zeroPad[12] = {0};

        // 0. Se a rajada contém o sub-pacote completo de temperatura no final, decodifica direto
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

        // 1. Padrão estrito (único em todos os quadros 35..60°C): '1000001' + p(2)+u(3) + '0001' + >=12 zeros
        for (size_t i = 0; i + 29 <= bitsLen; i++) {
            if (memcmp(&bits[i], syncPat, 7) == 0 &&
                bits[i + 12] == 0 && bits[i + 13] == 0 && bits[i + 14] == 0 && bits[i + 15] == 1) {
                bool hasZeroPad = false;
                size_t searchEnd = (i + 32 <= bitsLen) ? (i + 32) : bitsLen;
                for (size_t z = i + 16; z + 12 <= searchEnd; z++) {
                    if (memcmp(&bits[z], zeroPad, 12) == 0) {
                        hasZeroPad = true;
                        break;
                    }
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

        // 2. Fallback para rajadas parciais (ex: sem o bit 1 após '000'): busca na segunda metade da rajada
        size_t startSearch = (bitsLen > 140) ? (bitsLen / 2) : 0;
        for (size_t i = startSearch; i + 28 <= bitsLen; i++) {
            if (memcmp(&bits[i], syncPat, 7) == 0 &&
                bits[i + 12] == 0 && bits[i + 13] == 0 && bits[i + 14] == 0) {
                bool hasZeroPad = false;
                size_t searchEnd = (i + 38 <= bitsLen) ? (i + 38) : bitsLen;
                for (size_t z = i + 15; z + 12 <= searchEnd; z++) {
                    if (memcmp(&bits[z], zeroPad, 12) == 0) {
                        hasZeroPad = true;
                        break;
                    }
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
    uint8_t lastFullFrame[80] = {0};
    uint8_t lastFullFrameLen = 0;

    bool isOnline(uint32_t nowMs) const {
        return valid && ((nowMs - lastPacketMs) < 5000);
    }
};

/**
 * @brief Receptor UART passivo com detecção por preâmbulo, janela deslizante de chama e silêncio entre rajadas.
 */
class RinnaiUartDecoder {
public:
    static constexpr size_t BUF_CAP = 256;
    static constexpr size_t BURST_CAP = 96;
    static constexpr uint32_t INTERFRAME_GAP_MS = 8;
    static constexpr uint32_t FLAME_HOLD_TIMEOUT_MS = 650;

    bool feedByte(uint8_t b, uint32_t nowMs) {
        bool updated = false;

        // Se houve silêncio >= 8 ms desde o último byte, fecha a rajada anterior primeiro
        if (_burstLen > 0 && (nowMs - _lastByteMs) >= INTERFRAME_GAP_MS) {
            if (flushBurst(nowMs)) {
                updated = true;
            }
        }
        _lastByteMs = nowMs;

        // Contabilidade diagnóstica de bytes brutos
        _state.rxByteCount++;
        if (_state.lastRawLen < sizeof(_state.lastRawBytes)) {
            _state.lastRawBytes[_state.lastRawLen++] = b;
        } else {
            memmove(_state.lastRawBytes, _state.lastRawBytes + 1, sizeof(_state.lastRawBytes) - 1);
            _state.lastRawBytes[sizeof(_state.lastRawBytes) - 1] = b;
        }

        // Acumula na rajada atual
        if (_burstLen < BURST_CAP) {
            _burst[_burstLen++] = b;
        }

        // 1. Atualiza janela deslizante de 63 bytes para detectar 0x77 mesmo sem preâmbulo
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

        // 2. Registra ciclo completo entre preâmbulos consecutivos e acumula no buffer de alinhamento
        bool endsWithPreamble = (_state.lastRawLen >= 4) &&
                                RinnaiFrameParser::matchesPreamble(&_state.lastRawBytes[_state.lastRawLen - 4], 4);
        if (_cycleLen == 0) {
            if (endsWithPreamble) {
                memcpy(_cycleBuf, &_state.lastRawBytes[_state.lastRawLen - 4], 4);
                _cycleLen = 4;
            }
        } else {
            if (_cycleLen < sizeof(_cycleBuf)) {
                _cycleBuf[_cycleLen++] = b;
            }
            if (endsWithPreamble && _cycleLen >= 44) {
                size_t cycleBytes = _cycleLen - 4;
                if (cycleBytes > sizeof(_state.lastFullFrame)) {
                    cycleBytes = sizeof(_state.lastFullFrame);
                }
                memcpy(_state.lastFullFrame, _cycleBuf, cycleBytes);
                _state.lastFullFrameLen = (uint8_t)cycleBytes;
                memcpy(_cycleBuf, &_state.lastRawBytes[_state.lastRawLen - 4], 4);
                _cycleLen = 4;
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
            if (_bufLen < RinnaiFrameParser::FRAME_LEN) {
                break;
            }

            if (_state.lastFullFrameLen == 0) {
                size_t copyLen = (_bufLen < sizeof(_state.lastFullFrame)) ? _bufLen : sizeof(_state.lastFullFrame);
                memcpy(_state.lastFullFrame, _buf, copyLen);
                _state.lastFullFrameLen = (uint8_t)copyLen;
            }

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
                    if (_bufLen < neededLen) {
                        break;
                    }
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
            if (flushBurst(nowMs)) {
                updated = true;
            }
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
            if (RinnaiFrameParser::matchesPreamble(&_buf[i], _bufLen - i)) {
                return (int)i;
            }
        }
        return -1;
    }

    bool flushBurst(uint32_t nowMs) {
        size_t len = _burstLen;
        _burstLen = 0;
        if (len < 30) return false;

        _state.lastBurstSize = (uint8_t)len;

        // Se o alinhador por preâmbulo acabou de processar este ciclo, não duplica
        if (_lastAlignedMs != 0 && (nowMs - _lastAlignedMs) < 120) {
            return false;
        }

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
        _burstLen = 0; // Quadro já processado pelo alinhador estrito
        _state.packetCount++;
        _state.lastPacketMs = nowMs;
        _state.valid = true;
        memcpy(_state.rawTail, frame + (RinnaiFrameParser::FRAME_LEN - 12), 12);

        size_t cnt77 = RinnaiFrameParser::countFlameBytes(frame, RinnaiFrameParser::FRAME_LEN);
        if (cnt77 >= RinnaiFrameParser::FLAME_THRESHOLD) {
            _state.flameActive = true;
            _state.powerOn = true;
            _lastFlameMs = nowMs;
            _lastFlamePacketMs = nowMs;
            return;
        }

        _state.flameActive = false;
        bool isOff = RinnaiFrameParser::checkPanelOff(frame);
        _state.powerOn = !isOff;

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

    uint8_t _cycleBuf[84] = {0};
    size_t _cycleLen = 0;

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
