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
 * @brief Controlador não-bloqueante em malha fechada com trava anti-loop.
 *
 * - Limita o setpoint remoto à faixa segura de banho: 35°C a 48°C (passo 1°C).
 * - Quando há leitura de temperatura válida na linha TX, executa um clique por vez,
 *   aguarda o próximo quadro (~225 ms) e confere se a temperatura mudou.
 * - Trava anti-loop: após 2 cliques consecutivos sem alteração da temperatura real
 *   (ex: bloqueio anti-queimadura >42°C durante o banho ou painel travado/standby),
 *   aborta o ajuste e sincroniza o setpoint do HomeKit de volta para a temperatura real.
 */
class RinnaiClosedLoopController {
public:
    static constexpr uint8_t TEMP_MIN = 35;
    static constexpr uint8_t TEMP_MAX = 48;
    static constexpr uint32_t PRESS_DURATION_MS = 90;
    static constexpr uint32_t SETTLE_MIN_MS = 280;
    static constexpr uint32_t SETTLE_MAX_MS = 550;
    static constexpr uint8_t MAX_STALLED_CLICKS = 2;
    static constexpr uint8_t MAX_BLIND_CLICKS = 15;

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

        if (!telemetry.valid || telemetry.temperature == 0) {
            // Ainda não há telemetria inicial do barramento; guarda meta até sincronizar
            _targetTemp = clamped;
            _hasPendingTarget = true;
            _stalledClicks = 0;
            return;
        }

        if (!telemetry.powerOn) {
            // Painel fisicamente desligado (Standby): não envia cliques e re-sincroniza HomeKit
            _hasPendingTarget = false;
            _stalledClicks = 0;
            _blindClicksRemaining = 0;
            _targetTemp = clampTargetTemp(telemetry.temperature);
            notifySync(_targetTemp);
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
            int baseTemp = (_estimatedTemp >= 35 && _estimatedTemp <= 60)
                               ? _estimatedTemp
                               : telemetry.temperature;
            int diff = (int)_targetTemp - baseTemp;
            if (diff > MAX_BLIND_CLICKS) diff = MAX_BLIND_CLICKS;
            if (diff < -MAX_BLIND_CLICKS) diff = -MAX_BLIND_CLICKS;
            _blindClicksRemaining = (int8_t)diff;
            _reconcileAfterFlame = true;
            if (diff == 0) {
                _hasPendingTarget = false;
            }
        } else {
            _blindClicksRemaining = 0;
            _reconcileAfterFlame = false;
        }
        (void)nowMs;
    }

    void update(const RinnaiTelemetryState& telemetry, uint32_t nowMs) {
        // Sincroniza estimativa inicial na primeira leitura válida
        if (telemetry.valid && telemetry.temperature > 0 && _lastObservedTemp == 0) {
            _lastObservedTemp = telemetry.temperature;
            _estimatedTemp = telemetry.temperature;
            if (!_hasPendingTarget) {
                _targetTemp = clampTargetTemp(telemetry.temperature);
            }
        }

        // Detecta fim do banho (chama apagada) para reconciliar temperatura real após cliques em chama
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
                notifySync(realClamped);
            }
        } else if (telemetry.flameActive) {
            _wasFlameActive = true;
        }

        // Atualiza setpoint no Zigbee se usuário apertou o botão físico no painel em repouso
        if (_state == State::IDLE && !_hasPendingTarget &&
            telemetry.valid && !telemetry.flameActive && telemetry.powerOn &&
            telemetry.temperature > 0 && telemetry.temperature != _lastObservedTemp) {
            _lastObservedTemp = telemetry.temperature;
            _estimatedTemp = telemetry.temperature;
            uint8_t realClamped = clampTargetTemp(telemetry.temperature);
            _targetTemp = realClamped;
            notifySync(realClamped);
        }

        switch (_state) {
            case State::IDLE: {
                if (!_hasPendingTarget || !telemetry.valid || !telemetry.powerOn || telemetry.temperature == 0) {
                    return;
                }

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

                bool gotFreshTempPacket = (telemetry.tempPacketCount != _tempPktBeforeClick);
                if (!gotFreshTempPacket && elapsed < SETTLE_MAX_MS) {
                    return;
                }

                // Se temos leitura de temperatura real (repouso ou quadro decodificado durante chama)
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
                            // Trava anti-loop acionada!
                            _hasPendingTarget = false;
                            _stalledClicks = 0;
                            _blindClicksRemaining = 0;
                            _reconcileAfterFlame = false;
                            uint8_t realClamped = clampTargetTemp(telemetry.temperature);
                            _targetTemp = realClamped;
                            notifySync(realClamped);
                            _state = State::IDLE;
                            return;
                        }
                    }
                } else {
                    // Em chama ativa com quadros 0x77 sem trailer de temperatura
                    if (_blindClicksRemaining == 0) {
                        _hasPendingTarget = false;
                    }
                }

                _state = State::IDLE;
                break;
            }
        }
    }

    State state() const { return _state; }
    bool hasPendingTarget() const { return _hasPendingTarget; }
    uint8_t targetTemp() const { return _targetTemp; }
    uint8_t stalledClicks() const { return _stalledClicks; }

private:
    void notifySync(uint8_t temp) {
        if (_onSync) _onSync(temp, _cbCtx);
    }

    ITriStateHardware& _hw;
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
