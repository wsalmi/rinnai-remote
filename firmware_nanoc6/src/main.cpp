#include <Arduino.h>
#include "rinnai_uart_decoder.h"
#include "rinnai_tristate_buttons.h"
#include "rinnai_zigbee_thermostat.h"

// ==========================================
// PINAGEM M5NanoC6 (ESP32-C6)
// ==========================================
// Conector Grove (HY2.0-4P):
//   Preto    -> GND (Chicote Rinnai Fio 2)
//   Vermelho -> 5V  (Chicote Rinnai Fio 1)
//   Branco   -> G1  (TX Central Rinnai via divisor 10k/20k -> RX Passivo 2800 baud)
//   Amarelo  -> G2  (Saída Tri-State -> HY-M154 modificado para '+' e '-')
#define RINNAI_RX_PIN       1
#define RINNAI_TRISTATE_PIN 2

// Periféricos integrados do M5NanoC6:
#define BLUE_LED_PIN        7
#define BTN_BOOT_PIN        9
#define RGB_PWR_PIN         19
#define RGB_DATA_PIN        20

#define ZIGBEE_ENDPOINT_ID  1
#define RESET_HOLD_MS       5000

HardwareSerial RinnaiSerial(1);
RinnaiUartDecoder decoder;
Esp32TriStateHardware triStateHw(RINNAI_TRISTATE_PIN);
RinnaiClosedLoopController controller(triStateHw);
RinnaiZigbeeThermostat zbThermostat(ZIGBEE_ENDPOINT_ID);

static uint32_t clickFlashUntilMs = 0;
static RinnaiClickDir clickFlashDir = RinnaiClickDir::NONE;
static uint32_t blueBlinkUntilMs = 0;

static constexpr uint32_t UART_PIN_PROBE_TIMEOUT_MS = 2500;
static uint8_t activeRxPin = RINNAI_TRISTATE_PIN; // Inicia em G2 (maior atividade detectada em bancada)
static uint32_t lastPinSwitchMs = 0;
static uint32_t pktCountAtSwitch = 0;

static volatile uint32_t edgesG1 = 0;
static volatile uint32_t lastEdgeUsG1 = 0;
static volatile uint32_t lastPulseUsG1 = 0;
static volatile uint32_t edgesG2 = 0;
static volatile uint32_t lastEdgeUsG2 = 0;
static volatile uint32_t lastPulseUsG2 = 0;

void IRAM_ATTR onEdgeG1() {
    uint32_t now = (uint32_t)micros();
    if (lastEdgeUsG1 != 0) {
        lastPulseUsG1 = now - lastEdgeUsG1;
    }
    lastEdgeUsG1 = now;
    edgesG1++;
}

void IRAM_ATTR onEdgeG2() {
    uint32_t now = (uint32_t)micros();
    if (lastEdgeUsG2 != 0) {
        lastPulseUsG2 = now - lastEdgeUsG2;
    }
    lastEdgeUsG2 = now;
    edgesG2++;
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

    // Reanexa interrupções diagnósticas caso UART mux tenha reconfigurado o pino
    detachInterrupt(digitalPinToInterrupt(RINNAI_RX_PIN));
    detachInterrupt(digitalPinToInterrupt(RINNAI_TRISTATE_PIN));
    attachInterrupt(digitalPinToInterrupt(RINNAI_RX_PIN), onEdgeG1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RINNAI_TRISTATE_PIN), onEdgeG2, CHANGE);
}

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

static void updateStatusLeds(uint32_t nowMs, bool zbConnected, bool btnPressed, uint32_t btnHoldMs,
                             const RinnaiTelemetryState &telemetry) {
    // LED Azul discreto: pulso curto ao receber quadro válido
    setBlueLed(nowMs < blueBlinkUntilMs);

    // Feedback de Reset / Pareamento ao segurar botão G9
    if (btnPressed && btnHoldMs > 400) {
        if (btnHoldMs >= RESET_HOLD_MS) {
            setRgbColor(80, 0, 0); // Vermelho sólido antes de resetar
        } else {
            bool on = ((nowMs / 100) % 2) == 0;
            setRgbColor(on ? 50 : 0, 0, on ? 60 : 0); // Roxo piscando rápido
        }
        return;
    }

    // Piscada curta indicando clique '+' (Ciano) ou '-' (Magenta)
    if (nowMs < clickFlashUntilMs) {
        if (clickFlashDir == RinnaiClickDir::UP) {
            setRgbColor(0, 55, 55);
        } else {
            setRgbColor(55, 0, 55);
        }
        return;
    }

    // Não conectado ao Zigbee: azul piscando
    if (!zbConnected) {
        bool on = ((nowMs / 400) % 2) == 0;
        setRgbColor(0, 0, on ? 45 : 0);
        return;
    }

    // Conectado ao Zigbee: indica estado do aquecedor
    if (telemetry.flameActive) {
        setRgbColor(65, 22, 0); // Laranja = Chama ativa (banho)
    } else if (telemetry.isOnline(nowMs) && telemetry.powerOn) {
        setRgbColor(0, 20, 0);  // Verde suave = Ligado / Standby pronto
    } else if (telemetry.isOnline(nowMs) && !telemetry.powerOn) {
        setRgbColor(12, 12, 2); // Âmbar bem fraco = Painel em Off físico
    } else {
        setRgbColor(18, 12, 0); // Sem sinal recente da central
    }
}

static void handleResetButton(uint32_t nowMs) {
    static bool wasPressed = false;
    static uint32_t pressStartMs = 0;

    bool pressed = (digitalRead(BTN_BOOT_PIN) == LOW);
    if (pressed && !wasPressed) {
        pressStartMs = nowMs;
    } else if (pressed && (nowMs - pressStartMs) >= RESET_HOLD_MS) {
        Serial.println("[ZIGBEE] Botao G9 segurado por 5s -> Factory Reset & Pairing!");
        setRgbColor(80, 0, 0);
        delay(300);
        Zigbee.factoryReset(true);
    }
    wasPressed = pressed;

    updateStatusLeds(nowMs, Zigbee.connected(), pressed, pressed ? (nowMs - pressStartMs) : 0, decoder.state());
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n====================================================");
    Serial.println("  Rinnai Smart Remote - M5NanoC6 (Zigbee + HY-M154) ");
    Serial.println("====================================================");

    // Configurar GPIOs internos e Grove
    pinMode(BLUE_LED_PIN, OUTPUT);
    setBlueLed(false);
    pinMode(BTN_BOOT_PIN, INPUT_PULLUP);
    pinMode(RGB_PWR_PIN, OUTPUT);
    digitalWrite(RGB_PWR_PIN, HIGH); // Habilita alimentação do WS2812 no M5NanoC6
    setRgbColor(0, 0, 30);

    // Garante G2 em Alta Impedância (solto) desde o primeiro instante do boot
    triStateHw.begin();
    controller.setCallbacks(onControllerSetpointSync, onControllerClick, nullptr);

    // Inicia escuta passiva UART1 em G2 (alternando com G1 se não houver pacotes válidos em 2,5s)
    startPassiveUartOnPin(RINNAI_TRISTATE_PIN, millis(), false);
    Serial.printf("[UART] Escuta passiva iniciada em G%u @ 2800 baud (8N1, pull-up ativo, auto-probe G2<->G1)\n",
                  activeRxPin);
    Serial.printf("[CTRL] Atuador Tri-State configurado em G%d (HY-M154)\n", RINNAI_TRISTATE_PIN);

    // Configurar Zigbee Thermostat Endpoint
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

    Serial.println("[ZIGBEE] Iniciando stack Zigbee End Device...");
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
    Serial.println("[ZIGBEE] Stack iniciado. Aguardando pareamento / conexao...");
}

void loop() {
    uint32_t nowMs = millis();

    // 1. Ler todos os bytes disponíveis da UART passiva (G2 ou G1)
    bool telemetryUpdated = false;
    while (RinnaiSerial.available() > 0) {
        uint8_t b = (uint8_t)RinnaiSerial.read();
        if (decoder.feedByte(b, nowMs)) {
            telemetryUpdated = true;
            blueBlinkUntilMs = nowMs + 35;
        }
    }
    if (decoder.tick(nowMs)) {
        telemetryUpdated = true;
    }

    const RinnaiTelemetryState &telemetry = decoder.state();

    // Auto-detecção G2 <-> G1: se não houver pacote válido recente em 2,5s no pino atual, alterna
    bool pinHasRecentPacket = telemetry.valid &&
                                (telemetry.packetCount > pktCountAtSwitch) &&
                                ((nowMs - telemetry.lastPacketMs) < UART_PIN_PROBE_TIMEOUT_MS);
    if (!pinHasRecentPacket && (nowMs - lastPinSwitchMs) >= UART_PIN_PROBE_TIMEOUT_MS) {
        uint8_t nextPin = (activeRxPin == RINNAI_TRISTATE_PIN) ? RINNAI_RX_PIN : RINNAI_TRISTATE_PIN;
        Serial.printf("[UART] Sem pacotes validos em G%u apos %.1fs -> alternando para G%u\n",
                      activeRxPin, UART_PIN_PROBE_TIMEOUT_MS / 1000.0f, nextPin);
        startPassiveUartOnPin(nextPin, nowMs, true);
    }

    // 2. Processar comandos recebidos do Zigbee2MQTT / HomeKit
    int requestedSetpoint = 0;
    if (zbThermostat.consumePendingSetpoint(requestedSetpoint)) {
        Serial.printf("[ZIGBEE->CTRL] Novo alvo solicitado: %d°C\n", requestedSetpoint);
        controller.setTargetTemperature(requestedSetpoint, telemetry, nowMs);
        zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
    }

    if (zbThermostat.consumePendingModeRevert()) {
        Serial.printf("[ZIGBEE] Revertendo system_mode para estado fisico (%s)\n",
                      telemetry.powerOn ? "HEAT" : "OFF");
        zbThermostat.updateSystemMode(telemetry.powerOn, true);
    }

    // 3. Atualizar máquina de estados Tri-State (G2) somente quando G2 não estiver sendo escutado pela UART
    if (activeRxPin != RINNAI_TRISTATE_PIN) {
        controller.update(telemetry, nowMs);
    }

    // 4. Reportar telemetria ao Zigbee quando conectar ou houver mudança
    static bool wasZbConnected = false;
    static uint32_t lastPeriodicReportMs = 0;
    bool zbConnected = Zigbee.connected();
    bool justConnected = (zbConnected && !wasZbConnected);
    wasZbConnected = zbConnected;

    if (justConnected) {
        Serial.println("[ZIGBEE] Conectado a rede Zigbee! Enviando estado inicial...");
        zbThermostat.applySetpointLimits();
        lastPeriodicReportMs = nowMs;
        if (telemetry.valid) {
            zbThermostat.updateSystemMode(telemetry.powerOn, true);
            zbThermostat.updateRunningState(telemetry.flameActive, true);
            if (telemetry.temperature > 0) {
                zbThermostat.updateLocalTemperature(telemetry.temperature, true);
                zbThermostat.updateHeatingSetpoint(
                    RinnaiClosedLoopController::clampTargetTemp(telemetry.temperature), true
                );
            }
        }
    } else if (zbConnected && telemetry.valid) {
        bool periodicSync = (nowMs - lastPeriodicReportMs) >= 60000;
        if (telemetryUpdated || periodicSync) {
            zbThermostat.updateSystemMode(telemetry.powerOn, periodicSync);
            zbThermostat.updateRunningState(telemetry.flameActive, periodicSync);
            if (telemetry.temperature > 0) {
                zbThermostat.updateLocalTemperature(telemetry.temperature, periodicSync);
            }
            if (periodicSync) {
                zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
                lastPeriodicReportMs = nowMs;
            }
        }
    }

    // 5. Botão de Factory Reset (G9) e LEDs de status
    handleResetButton(nowMs);

    // 6. Log periódico Serial USB para diagnóstico
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
        Serial.printf("[STATUS] ZB=%s | RxPin=G%u | Online=%s | Power=%s | Chama=%s | Temp=%u°C | Alvo=%u°C | Pkts=%u | RxBytes=%u | Burst=%u | G1=%d(E=%u,%uus) | G2=%d(E=%u,%uus) | TailHEX=[%s] | LastHEX=[%s]\n",
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
                      gpio_get_level((gpio_num_t)RINNAI_RX_PIN),
                      (unsigned)edgesG1,
                      (unsigned)lastPulseUsG1,
                      gpio_get_level((gpio_num_t)RINNAI_TRISTATE_PIN),
                      (unsigned)edgesG2,
                      (unsigned)lastPulseUsG2,
                      tailHexStr,
                      rawHexStr);

        char fullFrameHex[256] = "--";
        formatHex(telemetry.lastFullFrame, telemetry.lastFullFrameLen, fullFrameHex, sizeof(fullFrameHex));
        Serial.printf("[FRAME] len=%u HEX=[%s]\n",
                      telemetry.lastFullFrameLen,
                      fullFrameHex);
    }

    delay(5);
}
