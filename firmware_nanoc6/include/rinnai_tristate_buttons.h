#pragma once
#include <stdint.h>
#include "rinnai_uart_decoder.h"

#ifdef ARDUINO
#include <Arduino.h>
#include "driver/gpio.h"
#endif

enum class RinnaiClickDir : uint8_t {
    NONE = 0,
    UP   = 1, // Botão '+' -> G2 = OUTPUT HIGH (3.3V -> Canal 1 -> GND)
    DOWN = 2  // Botão '-' -> G2 = OUTPUT LOW  (5.0V -> Canal 2 -> G2)
};

/**
 * @brief Interface abstrata de hardware para o atuador Tri-State em um único pino GPIO.
 */
class ITriStateHardware {
public:
    virtual ~ITriStateHardware() = default;
    virtual void release() = 0;
    virtual void driveUp() = 0;
    virtual void driveDown() = 0;
};

#ifdef ARDUINO
/**
 * @brief Implementação real ESP32-C6 no pino Grove G2 (GPIO 2) para módulo HY-M154 modificado.
 */
class Esp32TriStateHardware : public ITriStateHardware {
public:
    explicit Esp32TriStateHardware(uint8_t pin) : _pin(pin) {}

    void begin() {
        release();
    }

    void release() override {
        pinMode(_pin, INPUT);
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
    }

    void driveUp() override {
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
        digitalWrite(_pin, HIGH);
        pinMode(_pin, OUTPUT);
    }

    void driveDown() override {
        gpio_set_pull_mode((gpio_num_t)_pin, GPIO_FLOATING);
        digitalWrite(_pin, LOW);
        pinMode(_pin, OUTPUT);
    }

private:
    uint8_t _pin;
};
#endif

/**
 * @brief Controlador não-bloqueante em malha fechada (1 clique por vez) com wake-up reativo e trava anti-loop.
 *
 * - Limita o setpoint remoto à faixa segura de banho: 35°C a 43°C (passo 1°C).
 * - Executa 1 clique por vez e monitora a telemetria RMT (que usa debounce de 2 pacotes B3).
 *   Assim que a temperatura avança na direção esperada (após SETTLE_MIN_MS), prossegue imediatamente;
 *   caso contrário, aguarda até SETTLE_MAX_MS (1500 ms) sem sair prematuramente por pacotes antigos.
 * - Wake-up reativo do painel em Idle: se o 1º clique de um comando não alterar a temperatura
 *   dentro de SETTLE_MAX_MS, assume que apenas acendeu o display sem contar como stall.
 * - Trava anti-loop: após 2 cliques consecutivos sem mudança após o painel estar acordado
 *   (ex: bloqueio anti-queimadura >42°C durante o banho) ou ao atingir MAX_TOTAL_CLICKS_PER_CMD,
 *   aborta o ajuste e sincroniza o setpoint do HomeKit de volta para a temperatura real.
 */
class RinnaiClosedLoopController {
public:
    static constexpr uint8_t TEMP_MIN = 35;
    static constexpr uint8_t TEMP_MAX = 43;
    static constexpr uint32_t PRESS_DURATION_MS = 90;
    static constexpr uint32_t SETTLE_MIN_MS = 280;
    static constexpr uint32_t SETTLE_MAX_MS = 1500;
    static constexpr uint8_t MAX_STALLED_CLICKS = 2;
    static constexpr uint8_t MAX_TOTAL_CLICKS_PER_CMD = 20;

    enum class State : uint8_t {
        IDLE,
        PRESSING,
        WAITING_SETTLE
    };

    using SetpointSyncCallback = void (*)(uint8_t syncedTemp, void* ctx);
    using ClickCallback = void (*)(RinnaiClickDir dir, void* ctx);

    explicit RinnaiClosedLoopController(ITriStateHardware& hw) : _hw(hw) {
        _hw.release();
    }

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

    void setTargetTemperature(int requestedTempC, const RinnaiTelemetryState& telemetry, uint32_t nowMs) {
        uint8_t clamped = clampTargetTemp(requestedTempC);
        (void)nowMs;

        if (!telemetry.valid || telemetry.temperature == 0) {
            // Ainda não há telemetria inicial do barramento; guarda meta até sincronizar
            _targetTemp = clamped;
            _hasPendingTarget = true;
            _stalledClicks = 0;
            _totalClicksInCmd = 0;
            if (_state == State::IDLE) {
                _wakeClickUsed = false;
            }
            return;
        }

        if (!telemetry.powerOn) {
            // Painel fisicamente desligado (Standby): não envia cliques e re-sincroniza HomeKit
            _hasPendingTarget = false;
            _stalledClicks = 0;
            _wakeClickUsed = false;
            _totalClicksInCmd = 0;
            _targetTemp = clampTargetTemp(telemetry.temperature);
            notifySync(_targetTemp);
            return;
        }

        _targetTemp = clamped;
        _stalledClicks = 0;
        _totalClicksInCmd = 0;

        if (_state == State::IDLE) {
            _wakeClickUsed = false;
            if (telemetry.temperature == _targetTemp) {
                _hasPendingTarget = false;
                return;
            }
        }

        _hasPendingTarget = true;
    }

    void update(const RinnaiTelemetryState& telemetry, uint32_t nowMs) {
        // Sincroniza estimativa inicial e publica imediatamente ao Zigbee na primeira leitura válida
        if (telemetry.valid && telemetry.temperature > 0 && _lastObservedTemp == 0) {
            _lastObservedTemp = telemetry.temperature;
            if (!_hasPendingTarget) {
                _targetTemp = clampTargetTemp(telemetry.temperature);
                notifySync(_targetTemp);
            }
        }

        // Atualiza setpoint no Zigbee se usuário apertou o botão físico no painel (com ou sem chama)
        if (_state == State::IDLE && !_hasPendingTarget &&
            telemetry.valid && telemetry.powerOn &&
            telemetry.temperature > 0 && telemetry.temperature != _lastObservedTemp) {
            _lastObservedTemp = telemetry.temperature;
            uint8_t realClamped = clampTargetTemp(telemetry.temperature);
            _targetTemp = realClamped;
            notifySync(realClamped);
        }

        switch (_state) {
            case State::IDLE: {
                if (!_hasPendingTarget || !telemetry.valid || telemetry.temperature == 0) {
                    return;
                }
                if (!telemetry.powerOn) {
                    abortAndSync(telemetry.temperature);
                    return;
                }

                if (telemetry.temperature == _targetTemp) {
                    _hasPendingTarget = false;
                    _stalledClicks = 0;
                    _wakeClickUsed = false;
                    _totalClicksInCmd = 0;
                    _lastObservedTemp = telemetry.temperature;
                    return;
                }

                if (_totalClicksInCmd >= MAX_TOTAL_CLICKS_PER_CMD) {
                #ifdef ARDUINO
                    Serial.printf("[CTRL] Limite maximo de cliques (%u) atingido em %u°C -> abortando\n",
                                  (unsigned)MAX_TOTAL_CLICKS_PER_CMD, telemetry.temperature);
                #endif
                    abortAndSync(telemetry.temperature);
                    return;
                }

                RinnaiClickDir dir = (_targetTemp > telemetry.temperature) ? RinnaiClickDir::UP : RinnaiClickDir::DOWN;

                _activeDir = dir;
                _tempBeforeClick = telemetry.temperature;
                _stateStartMs = nowMs;
                _state = State::PRESSING;
                _totalClicksInCmd++;

                if (dir == RinnaiClickDir::UP) {
                    _hw.driveUp();
                } else {
                    _hw.driveDown();
                }
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
                if (elapsed < SETTLE_MIN_MS) {
                    return;
                }

                if (!_hasPendingTarget) {
                    _state = State::IDLE;
                    return;
                }

                if (!telemetry.valid || telemetry.temperature == 0) {
                    if (elapsed >= SETTLE_MAX_MS) {
                        _state = State::IDLE;
                    }
                    return;
                }

                if (!telemetry.powerOn) {
                    abortAndSync(telemetry.temperature);
                    return;
                }

                // Confirmação rápida de chegada (exige mudança real em relação ao valor antes do clique)
                if (telemetry.temperature == _targetTemp && telemetry.temperature != _tempBeforeClick) {
                #ifdef ARDUINO
                    Serial.printf("[CTRL] Alvo %u°C atingido (cliques=%u)\n",
                                  _targetTemp, (unsigned)_totalClicksInCmd);
                #endif
                    _lastObservedTemp = telemetry.temperature;
                    _hasPendingTarget = false;
                    _stalledClicks = 0;
                    _wakeClickUsed = false;
                    _totalClicksInCmd = 0;
                    _state = State::IDLE;
                    return;
                }

                // Confirmação rápida de passo intermediário na direção esperada
                bool movedCorrectly =
                    (_activeDir == RinnaiClickDir::UP && telemetry.temperature > _tempBeforeClick) ||
                    (_activeDir == RinnaiClickDir::DOWN && telemetry.temperature < _tempBeforeClick);

                if (movedCorrectly) {
                #ifdef ARDUINO
                    Serial.printf("[CTRL] Passo confirmado: %u°C -> %u°C (alvo=%u°C)\n",
                                  _tempBeforeClick, telemetry.temperature, _targetTemp);
                #endif
                    _lastObservedTemp = telemetry.temperature;
                    _stalledClicks = 0;
                    _wakeClickUsed = true;
                    _state = State::IDLE;
                    return;
                }

                // Aguarda pacientemente o debounce RMT (2 pacotes B3) + latência painel->central
                if (elapsed < SETTLE_MAX_MS) {
                    return;
                }

                _lastObservedTemp = telemetry.temperature;

                // Caso o alvo tenha sido revertido durante o clique para o valor atual sem movimento
                if (telemetry.temperature == _targetTemp) {
                    _hasPendingTarget = false;
                    _stalledClicks = 0;
                    _wakeClickUsed = false;
                    _totalClicksInCmd = 0;
                    _state = State::IDLE;
                    return;
                }

                // Primeiro clique sem alteração assume despertar do painel em Idle (sem contar como stall)
                if (!_wakeClickUsed) {
                    _wakeClickUsed = true;
                #ifdef ARDUINO
                    Serial.printf("[CTRL] 1o clique sem mudanca em %u°C (painel Idle despertado); prosseguindo para %u°C\n",
                                  telemetry.temperature, _targetTemp);
                #endif
                    _state = State::IDLE;
                    return;
                }

                _stalledClicks++;
                if (_stalledClicks >= MAX_STALLED_CLICKS) {
                #ifdef ARDUINO
                    Serial.printf("[CTRL] Trava anti-loop (%u/%u sem mudanca em %u°C) -> revertendo alvo\n",
                                  (unsigned)_stalledClicks, (unsigned)MAX_STALLED_CLICKS, telemetry.temperature);
                #endif
                    abortAndSync(telemetry.temperature);
                    return;
                }

                #ifdef ARDUINO
                Serial.printf("[CTRL] Clique sem mudanca em %u°C (stall %u/%u); tentando novamente\n",
                              telemetry.temperature, (unsigned)_stalledClicks, (unsigned)MAX_STALLED_CLICKS);
                #endif
                _state = State::IDLE;
                break;
            }
        }
    }

    State state() const { return _state; }
    bool hasPendingTarget() const { return _hasPendingTarget; }
    uint8_t targetTemp() const { return _targetTemp; }
    uint8_t stalledClicks() const { return _stalledClicks; }
    bool wakeClickUsed() const { return _wakeClickUsed; }
    uint8_t totalClicksInCmd() const { return _totalClicksInCmd; }

private:
    void notifySync(uint8_t temp) {
        if (_onSync) _onSync(temp, _cbCtx);
    }

    void abortAndSync(uint8_t realTemp) {
        _hasPendingTarget = false;
        _stalledClicks = 0;
        _wakeClickUsed = false;
        _totalClicksInCmd = 0;
        uint8_t realClamped = clampTargetTemp(realTemp);
        _targetTemp = realClamped;
        _lastObservedTemp = realTemp;
        notifySync(realClamped);
        _state = State::IDLE;
    }

    ITriStateHardware& _hw;
    State _state = State::IDLE;
    uint32_t _stateStartMs = 0;

    RinnaiClickDir _activeDir = RinnaiClickDir::NONE;
    uint8_t _targetTemp = 40;
    uint8_t _tempBeforeClick = 0;
    uint8_t _lastObservedTemp = 0;

    bool _hasPendingTarget = false;
    bool _wakeClickUsed = false;
    uint8_t _stalledClicks = 0;
    uint8_t _totalClicksInCmd = 0;

    SetpointSyncCallback _onSync = nullptr;
    ClickCallback _onClick = nullptr;
    void* _cbCtx = nullptr;
};
