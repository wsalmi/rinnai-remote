#include <Arduino.h>
#include "rinnai_decoder.h"
#include "rinnai_encoder.h"

#define RINNAI_RX_PIN 5
#define RINNAI_TX_PIN 15

RinnaiDecoder decoder(RINNAI_RX_PIN);
RinnaiEncoder encoder(RINNAI_TX_PIN);

void IRAM_ATTR onPinChange() {
    uint32_t now = micros();
    bool level = digitalRead(RINNAI_RX_PIN);
    decoder.handleInterrupt(now, level);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n==========================================");
    Serial.println("   Rinnai Water Heater ESP32 Interface   ");
    Serial.println("==========================================");

    decoder.begin();
    encoder.begin();

    attachInterrupt(digitalPinToInterrupt(RINNAI_RX_PIN), onPinChange, CHANGE);
    Serial.printf("[INFO] Monitorando central no pino RX (GPIO %d)\n", RINNAI_RX_PIN);
    Serial.printf("[INFO] Transmissor de comandos pronto no pino TX (GPIO %d)\n", RINNAI_TX_PIN);
}

void loop() {
    // Processamento de recepção de telemetria
    if (decoder.hasPacket()) {
        RinnaiStatus status = decoder.getStatus();
        if (status.valid) {
            Serial.printf("[RINNAI] Status: %s | Chama: %s | Temp: %d°C | Diag: 0x%02X | RAW: ",
                          status.powerOn ? "LIGADO" : "STANDBY",
                          status.flameActive ? "ATIVA" : "APAGADA",
                          status.temperature,
                          status.diagCode);
            for (int i = 0; i < 6; i++) {
                Serial.printf("%02X ", status.rawBytes[i]);
            }
            Serial.println();
        }
    }

    // Leitura de comandos via Serial para testes de bancada
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case '+':
                Serial.println("[CMD] Enviando clique: Temperatura +");
                encoder.pressButton(RinnaiButton::TEMP_UP);
                break;
            case '-':
                Serial.println("[CMD] Enviando clique: Temperatura -");
                encoder.pressButton(RinnaiButton::TEMP_DOWN);
                break;
            case 'p':
            case 'P':
                Serial.println("[CMD] Enviando clique: Power On/Off");
                encoder.pressButton(RinnaiButton::POWER_TOGGLE);
                break;
        }
    }
}
