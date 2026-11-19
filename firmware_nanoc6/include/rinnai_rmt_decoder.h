#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "rinnai_uart_decoder.h"

#ifdef ARDUINO
#include <Arduino.h>
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#endif

struct RinnaiRmtPulse {
    uint16_t durationUs = 0;
    uint8_t level = 0;
};

struct RinnaiRmtBurstSummary {
    uint16_t pulseCount = 0;
    uint16_t highCount = 0;
    uint16_t lowCount = 0;
    uint16_t minHighUs = 0;
    uint16_t maxHighUs = 0;
    uint16_t minLowUs = 0;
    uint16_t maxLowUs = 0;
    uint8_t lowBitCount = 0;
    uint8_t highBitCount = 0;
    uint8_t pkt8[8] = {0};
    bool pkt8Valid = false;
    bool crcOk = false;
    bool inverted = false;
    uint32_t gapBeforeUs = 0;
    uint32_t timestampMs = 0;
};

class RinnaiRmtDecoder {
public:
    static constexpr uint16_t MIN_VALID_PULSE_US = 80;
    static constexpr uint16_t BIT_THRESHOLD_US = 700;
    static constexpr uint8_t TEMP_DEBOUNCE_COUNT = 2;
    static constexpr uint32_t FLAME_HOLD_TIMEOUT_MS = 1000;
    static constexpr size_t RECENT_BURST_SLOTS = 4;

    static uint16_t crc16Rinnai(const uint8_t *data, size_t len) {
        uint16_t crc = 0x0000;
        for (size_t i = 0; i < len; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t b = 0; b < 8; b++) {
                crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0xE815) : (uint16_t)(crc << 1);
            }
        }
        return crc;
    }

    bool feedPulses(const RinnaiRmtPulse *pulses, size_t count, uint32_t nowMs) {
        if (!pulses || count == 0) return false;

        RinnaiRmtBurstSummary summary;
        summary.pulseCount = (uint16_t)count;
        summary.timestampMs = nowMs;
        summary.gapBeforeUs = (_lastBurstMs > 0 && nowMs >= _lastBurstMs)
                                  ? (uint32_t)(nowMs - _lastBurstMs) * 1000U
                                  : 0U;
        _lastBurstMs = nowMs;

        uint16_t minH = 0xFFFF, maxH = 0;
        uint16_t minL = 0xFFFF, maxL = 0;
        uint8_t lowBits[160] = {0};
        uint8_t highBits[160] = {0};
        size_t lBits = 0;
        size_t hBits = 0;

        for (size_t i = 0; i < count; i++) {
            uint16_t dur = pulses[i].durationUs;
            uint8_t lvl = pulses[i].level ? 1 : 0;
            if (lvl == 1) {
                summary.highCount++;
                if (dur < minH) minH = dur;
                if (dur > maxH) maxH = dur;
                if (dur >= MIN_VALID_PULSE_US && hBits < sizeof(highBits)) {
                    highBits[hBits++] = (dur >= BIT_THRESHOLD_US) ? 1 : 0;
                }
            } else {
                summary.lowCount++;
                if (dur < minL) minL = dur;
                if (dur > maxL) maxL = dur;
                if (dur >= MIN_VALID_PULSE_US && lBits < sizeof(lowBits)) {
                    lowBits[lBits++] = (dur >= BIT_THRESHOLD_US) ? 1 : 0;
                }
            }
        }
        summary.minHighUs = (summary.highCount > 0) ? minH : 0;
        summary.maxHighUs = maxH;
        summary.minLowUs = (summary.lowCount > 0) ? minL : 0;
        summary.maxLowUs = maxL;
        summary.lowBitCount = (uint8_t)((lBits > 255) ? 255 : lBits);
        summary.highBitCount = (uint8_t)((hBits > 255) ? 255 : hBits);

        const uint8_t *chosenBits = nullptr;
        if (lBits == 64) {
            chosenBits = lowBits;
            summary.inverted = false;
        } else if (lBits < 64 && hBits == 64) {
            chosenBits = highBits;
            summary.inverted = true;
        }

        if (chosenBits != nullptr) {
            for (size_t byteIdx = 0; byteIdx < 8; byteIdx++) {
                uint8_t v = 0;
                for (size_t bit = 0; bit < 8; bit++) {
                    v = (uint8_t)((v << 1) | (chosenBits[byteIdx * 8 + bit] & 0x01));
                }
                summary.pkt8[byteIdx] = v;
            }
            summary.pkt8Valid = true;
            uint16_t expectedCrc = ((uint16_t)summary.pkt8[6] << 8) | summary.pkt8[7];
            uint16_t calcCrc = crc16Rinnai(summary.pkt8, 6);
            summary.crcOk = (calcCrc == expectedCrc);
            if (summary.crcOk) {
                _crcOkCount++;
            } else {
                _crcErrCount++;
            }
        }

        _recentBursts[_recentWriteIdx] = summary;
        _recentWriteIdx = (_recentWriteIdx + 1) % RECENT_BURST_SLOTS;
        _totalBursts++;
        _lastAnyBurst = summary;

        if (summary.pkt8Valid && summary.crcOk && ((summary.pkt8[0] & 0xF0) == 0x00)) {
            _lastB3Burst = summary;
            _state.valid = true;
            _state.packetCount++;
            _state.lastPacketMs = nowMs;
            _state.powerOn = true;
            memset(_state.rawTail, 0, sizeof(_state.rawTail));
            memcpy(_state.rawTail, summary.pkt8, 8);

            bool flame = ((summary.pkt8[0] & 0x04) != 0) || ((summary.pkt8[1] & 0xE0) != 0);
            _state.flameActive = flame;
            if (flame) {
                _lastFlameMs = nowMs;
            }

            int temp = (int)summary.pkt8[2];
            if (temp >= 35 && temp <= 60) {
                updateDebouncedTemp(temp);
            }
            return true;
        }
        return false;
    }

    bool tick(uint32_t nowMs) {
        if (_state.flameActive && _lastFlameMs != 0 && (nowMs - _lastFlameMs) > FLAME_HOLD_TIMEOUT_MS) {
            _state.flameActive = false;
            return true;
        }
        return false;
    }

    const RinnaiTelemetryState &state() const { return _state; }
    uint32_t totalBursts() const { return _totalBursts; }
    uint32_t crcOkCount() const { return _crcOkCount; }
    uint32_t crcErrCount() const { return _crcErrCount; }

    const RinnaiRmtBurstSummary &recentBurst(size_t i) const {
        size_t idx = (_recentWriteIdx + i) % RECENT_BURST_SLOTS;
        return _recentBursts[idx];
    }

    const RinnaiRmtBurstSummary &lastB3Burst() const { return _lastB3Burst; }
    const RinnaiRmtBurstSummary &lastAnyBurst() const { return _lastAnyBurst; }

private:
    void updateDebouncedTemp(int temp) {
        if (temp < 35 || temp > 60) return;
        uint8_t t = (uint8_t)temp;
        if (_state.temperature == 0) {
            _state.temperature = t;
            _state.tempPacketCount++;
            _pendingTemp = 0;
            _pendingTempCount = 0;
            return;
        }
        if (t == _state.temperature) {
            _pendingTemp = 0;
            _pendingTempCount = 0;
            _state.tempPacketCount++;
            return;
        }
        if (t == _pendingTemp) {
            _pendingTempCount++;
        } else {
            _pendingTemp = t;
            _pendingTempCount = 1;
        }
        if (_pendingTempCount >= TEMP_DEBOUNCE_COUNT) {
            _state.temperature = t;
            _state.tempPacketCount++;
            _pendingTemp = 0;
            _pendingTempCount = 0;
        }
    }

    RinnaiTelemetryState _state;
    RinnaiRmtBurstSummary _recentBursts[RECENT_BURST_SLOTS];
    RinnaiRmtBurstSummary _lastB3Burst;
    RinnaiRmtBurstSummary _lastAnyBurst;
    size_t _recentWriteIdx = 0;
    uint32_t _totalBursts = 0;
    uint32_t _crcOkCount = 0;
    uint32_t _crcErrCount = 0;
    uint32_t _lastBurstMs = 0;
    uint32_t _lastFlameMs = 0;
    uint8_t _pendingTemp = 0;
    uint8_t _pendingTempCount = 0;
};

#ifdef ARDUINO
class Esp32RmtReceiver {
public:
    static constexpr size_t MAX_SYMBOLS = 192;
    static constexpr size_t RING_SLOTS = 8;

    struct RawBurst {
        rmt_symbol_word_t symbols[MAX_SYMBOLS];
        size_t count = 0;
        uint32_t timestampMs = 0;
    };

    bool begin(int pin) {
        _pin = pin;

        rmt_rx_channel_config_t rx_cfg = {};
        rx_cfg.gpio_num = (gpio_num_t)pin;
        rx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
        rx_cfg.resolution_hz = 1000000; // 1 us por tick
        rx_cfg.mem_block_symbols = 48;
        rx_cfg.intr_priority = 0;

        esp_err_t err = rmt_new_rx_channel(&rx_cfg, &_chan);
        if (err != ESP_OK) {
            Serial.printf("[RMT] Falha rmt_new_rx_channel: 0x%x (%s)\n", err, esp_err_to_name(err));
            return false;
        }

        _evtQueue = xQueueCreate(8, sizeof(size_t));
        if (!_evtQueue) {
            Serial.println("[RMT] Falha ao criar fila de eventos");
            return false;
        }

        rmt_rx_event_callbacks_t cbs = {};
        cbs.on_recv_done = onRecvDone;
        err = rmt_rx_register_event_callbacks(_chan, &cbs, this);
        if (err != ESP_OK) {
            Serial.printf("[RMT] Falha rmt_rx_register_event_callbacks: 0x%x (%s)\n", err, esp_err_to_name(err));
            return false;
        }

        err = rmt_enable(_chan);
        if (err != ESP_OK) {
            Serial.printf("[RMT] Falha rmt_enable: 0x%x (%s)\n", err, esp_err_to_name(err));
            return false;
        }

        gpio_pulldown_dis((gpio_num_t)pin);
        gpio_pullup_en((gpio_num_t)pin);

        _rxCfg = {};
        _rxCfg.signal_range_min_ns = 3000;    // <= 3187 ns no ESP32-C6 @ 80MHz
        _rxCfg.signal_range_max_ns = 3000000; // 3000 us de silêncio encerra a rajada
        _rxCfg.flags.en_partial_rx = 0;

        _activeBuf = 0;
        err = rmt_receive(_chan, _dmaBuf[0], sizeof(_dmaBuf[0]), &_rxCfg);
        if (err != ESP_OK) {
            Serial.printf("[RMT] Falha rmt_receive inicial: 0x%x (%s)\n", err, esp_err_to_name(err));
            return false;
        }

        BaseType_t ok = xTaskCreate(rxTaskEntry, "rinnai_rmt_rx", 4096, this, 5, &_rxTask);
        if (ok != pdPASS) {
            Serial.println("[RMT] Falha ao criar task RX");
            return false;
        }

        return true;
    }

    bool popPulses(RinnaiRmtPulse *outPulses, size_t maxPulses, size_t &outCount, uint32_t &outTimestampMs) {
        if (!outPulses || maxPulses == 0) return false;

        RawBurst burst;
        bool hasItem = false;
        portENTER_CRITICAL(&_mux);
        if (_ringCount > 0) {
            burst = _ring[_ringTail];
            _ringTail = (_ringTail + 1) % RING_SLOTS;
            _ringCount--;
            hasItem = true;
        }
        portEXIT_CRITICAL(&_mux);

        if (!hasItem) return false;

        size_t idx = 0;
        for (size_t i = 0; i < burst.count && idx < maxPulses; i++) {
            const rmt_symbol_word_t &sym = burst.symbols[i];
            if (sym.duration0 > 0 && idx < maxPulses) {
                outPulses[idx].durationUs = sym.duration0;
                outPulses[idx].level = (uint8_t)sym.level0;
                idx++;
            }
            if (sym.duration1 > 0 && idx < maxPulses) {
                outPulses[idx].durationUs = sym.duration1;
                outPulses[idx].level = (uint8_t)sym.level1;
                idx++;
            }
        }
        outCount = idx;
        outTimestampMs = burst.timestampMs;
        return idx > 0;
    }

    uint32_t dropCount() const { return _dropCount; }
    uint32_t rxErrCount() const { return _rxErrCount; }

private:
    static bool IRAM_ATTR onRecvDone(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx) {
        (void)rx_chan;
        Esp32RmtReceiver *self = static_cast<Esp32RmtReceiver *>(user_ctx);
        BaseType_t highTaskWoken = pdFALSE;
        size_t numSym = edata ? edata->num_symbols : 0;
        if (self && self->_evtQueue) {
            xQueueSendFromISR(self->_evtQueue, &numSym, &highTaskWoken);
        }
        return (highTaskWoken == pdTRUE);
    }

    static void rxTaskEntry(void *arg) {
        static_cast<Esp32RmtReceiver *>(arg)->rxTaskLoop();
    }

    void rxTaskLoop() {
        size_t numSymbols = 0;
        for (;;) {
            if (xQueueReceive(_evtQueue, &numSymbols, portMAX_DELAY) == pdTRUE) {
                size_t doneBuf = _activeBuf;
                size_t nextBuf = _activeBuf ^ 1U;
                _activeBuf = nextBuf;

                esp_err_t err = rmt_receive(_chan, _dmaBuf[nextBuf], sizeof(_dmaBuf[nextBuf]), &_rxCfg);
                if (err != ESP_OK) {
                    _rxErrCount++;
                }

                if (numSymbols > MAX_SYMBOLS) numSymbols = MAX_SYMBOLS;
                uint32_t ts = (uint32_t)millis();

                portENTER_CRITICAL(&_mux);
                if (_ringCount >= RING_SLOTS) {
                    _ringTail = (_ringTail + 1) % RING_SLOTS;
                    _ringCount--;
                    _dropCount++;
                }
                RawBurst &slot = _ring[_ringHead];
                slot.count = numSymbols;
                slot.timestampMs = ts;
                if (numSymbols > 0) {
                    memcpy(slot.symbols, _dmaBuf[doneBuf], numSymbols * sizeof(rmt_symbol_word_t));
                }
                _ringHead = (_ringHead + 1) % RING_SLOTS;
                _ringCount++;
                portEXIT_CRITICAL(&_mux);
            }
        }
    }

    int _pin = -1;
    rmt_channel_handle_t _chan = nullptr;
    rmt_receive_config_t _rxCfg = {};
    QueueHandle_t _evtQueue = nullptr;
    TaskHandle_t _rxTask = nullptr;

    rmt_symbol_word_t _dmaBuf[2][MAX_SYMBOLS] = {};
    volatile size_t _activeBuf = 0;

    RawBurst _ring[RING_SLOTS] = {};
    size_t _ringHead = 0;
    size_t _ringTail = 0;
    size_t _ringCount = 0;
    portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

    volatile uint32_t _dropCount = 0;
    volatile uint32_t _rxErrCount = 0;
};
#endif
