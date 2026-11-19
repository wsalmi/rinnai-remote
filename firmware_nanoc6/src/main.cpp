#include <Arduino.h>
#include "rinnai_rmt_decoder.h"
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
#define HEATING_HOLD_MS     1500

// Estado exposto ao Zigbee: chama acesa = Aquecer (heat); qualquer outro caso = Desligado (off)
static uint32_t lastFlameSeenMs = 0;
static bool isHeating(const RinnaiTelemetryState &t, uint32_t nowMs) {
    if (t.flameActive) lastFlameSeenMs = nowMs;
    return lastFlameSeenMs != 0 && (nowMs - lastFlameSeenMs) < HEATING_HOLD_MS;
}
#define RESET_HOLD_MS       5000

Esp32RmtReceiver rmtRx;
RinnaiRmtDecoder decoder;
Esp32TriStateHardware triStateHw(RINNAI_TRISTATE_PIN);
RinnaiClosedLoopController controller(triStateHw);
RinnaiZigbeeThermostat zbThermostat(ZIGBEE_ENDPOINT_ID);

static uint32_t clickFlashUntilMs = 0;
static RinnaiClickDir clickFlashDir = RinnaiClickDir::NONE;
static uint32_t blueBlinkUntilMs = 0;

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
    if (isHeating(telemetry, nowMs)) {
        setRgbColor(65, 22, 0); // Laranja = Chama ativa (Aquecer)
    } else if (telemetry.isOnline(nowMs)) {
        setRgbColor(0, 12, 0);  // Verde fraco = Desligado (sem chama), telemetria ok
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

    // Inicia captura PWM hardware via RMT RX em G1 (1us/tick)
    if (rmtRx.begin(RINNAI_RX_PIN)) {
        Serial.println("[RMT] Captura PWM hardware iniciada em G1 (1us/tick, idle=3000us)");
    } else {
        Serial.println("[RMT] ERRO ao iniciar captura PWM em G1!");
    }
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
    zbThermostat.requestOTAUpdate();
    Serial.printf("[OTA] Firmware v0x%08X (Zigbee OTA cliente ativo)\n", (unsigned)RINNAI_FW_VERSION);
    Serial.println("[ZIGBEE] Stack iniciado. Aguardando pareamento / conexao...");
}

void loop() {
    uint32_t nowMs = millis();

    // 1. Drenar todas as rajadas PWM recebidas via RMT RX (G1)
    bool telemetryUpdated = false;
    static RinnaiRmtPulse pulseBuf[Esp32RmtReceiver::MAX_SYMBOLS * 2];
    size_t pulseCount = 0;
    uint32_t burstTsMs = 0;
    while (rmtRx.popPulses(pulseBuf, sizeof(pulseBuf) / sizeof(pulseBuf[0]), pulseCount, burstTsMs)) {
        if (decoder.feedPulses(pulseBuf, pulseCount, burstTsMs)) {
            telemetryUpdated = true;
            blueBlinkUntilMs = nowMs + 35;
        }
    }
    if (decoder.tick(nowMs)) {
        telemetryUpdated = true;
    }

    const RinnaiTelemetryState &telemetry = decoder.state();

    // 2. Processar comandos recebidos do Zigbee2MQTT / HomeKit
    int requestedSetpoint = 0;
    if (zbThermostat.consumePendingSetpoint(requestedSetpoint)) {
        Serial.printf("[ZIGBEE->CTRL] Novo alvo solicitado: %d°C\n", requestedSetpoint);
        controller.setTargetTemperature(requestedSetpoint, telemetry, nowMs);
        zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
    }

    if (zbThermostat.consumePendingModeRevert()) {
        Serial.printf("[ZIGBEE] Revertendo system_mode para estado fisico (%s)\n",
                      isHeating(telemetry, nowMs) ? "HEAT" : "OFF");
        zbThermostat.updateSystemMode(isHeating(telemetry, nowMs), true);
    }

    // 3. Atualizar máquina de estados Tri-State (G2)
    controller.update(telemetry, nowMs);

    // 4. Reportar telemetria ao Zigbee quando conectar ou houver mudança
    static bool wasZbConnected = false;
    static bool initialStateSynced = false;
    static uint32_t lastPeriodicReportMs = 0;
    bool zbConnected = Zigbee.connected();
    bool justConnected = (zbConnected && !wasZbConnected);
    bool heating = isHeating(telemetry, nowMs);
    static bool lastHeating = false;
    if (heating != lastHeating) { lastHeating = heating; telemetryUpdated = true; }
    wasZbConnected = zbConnected;

    if (justConnected) {
        Serial.println("[ZIGBEE] Conectado a rede Zigbee! Enviando estado inicial...");
        zbThermostat.applySetpointLimits();
        lastPeriodicReportMs = nowMs;
        initialStateSynced = false;
    }

    if (zbConnected && telemetry.valid) {
        bool periodicSync = !initialStateSynced || (nowMs - lastPeriodicReportMs) >= 60000;
        if (telemetryUpdated || periodicSync) {
            zbThermostat.updateSystemMode(heating, periodicSync);
            zbThermostat.updateRunningState(heating, periodicSync);
            if (telemetry.temperature > 0) {
                zbThermostat.updateLocalTemperature(telemetry.temperature, periodicSync);
            }
            if (periodicSync) {
                zbThermostat.updateHeatingSetpoint(controller.targetTemp(), true);
                lastPeriodicReportMs = nowMs;
                initialStateSynced = true;
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
        char b3HexStr[32] = "--";
        if (telemetry.valid) {
            formatHex(telemetry.rawTail, 8, b3HexStr, sizeof(b3HexStr));
        }
        Serial.printf("[STATUS] ZB=%s | Rx=RMT(G%d) | Online=%s | Modo=%s | Chama=%s | Temp=%u°C | Alvo=%u°C | Pkts=%u | CRC=%u/%u | Bursts=%u | Drops=%u | G1=%d | G2=%d | B3=[%s]\n",
                      zbConnected ? "OK" : "PAIRING",
                      RINNAI_RX_PIN,
                      telemetry.isOnline(nowMs) ? "YES" : "NO",
                      heating ? "HEAT" : "OFF",
                      telemetry.flameActive ? "ATIVA" : "IDLE",
                      telemetry.temperature,
                      controller.targetTemp(),
                      telemetry.packetCount,
                      (unsigned)decoder.crcOkCount(),
                      (unsigned)decoder.crcErrCount(),
                      (unsigned)decoder.totalBursts(),
                      (unsigned)rmtRx.dropCount(),
                      gpio_get_level((gpio_num_t)RINNAI_RX_PIN),
                      gpio_get_level((gpio_num_t)RINNAI_TRISTATE_PIN),
                      b3HexStr);

        for (size_t bi = 0; bi < RinnaiRmtDecoder::RECENT_BURST_SLOTS; bi++) {
            const RinnaiRmtBurstSummary &rb = decoder.recentBurst(bi);
            if (rb.pulseCount == 0) continue;
            char pkt8Str[32] = "--";
            if (rb.pkt8Valid) {
                formatHex(rb.pkt8, 8, pkt8Str, sizeof(pkt8Str));
            }
            Serial.printf("[RMT-B%u] n=%u H=%u..%uus(%u) L=%u..%uus(%u) lbits=%u hbits=%u inv=%d crc=%s pkt8=[%s]\n",
                          (unsigned)bi,
                          (unsigned)rb.pulseCount,
                          (unsigned)rb.minHighUs,
                          (unsigned)rb.maxHighUs,
                          (unsigned)rb.highCount,
                          (unsigned)rb.minLowUs,
                          (unsigned)rb.maxLowUs,
                          (unsigned)rb.lowCount,
                          (unsigned)rb.lowBitCount,
                          (unsigned)rb.highBitCount,
                          rb.inverted ? 1 : 0,
                          rb.crcOk ? "OK" : "ERR",
                          pkt8Str);
        }
    }

    delay(2);
}
