#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// ==========================================
// CONFIGURAÇÃO DE REDE WI-FI
// ==========================================
const char* WIFI_SSID = "Coio";
const char* WIFI_PASS = "Xwtuamio@0209";

// ==========================================
// PINAGEM NO ESP32-C3 SUPER MINI (HW-466AB)
// ==========================================
// Pinos recomendados no conector do Super Mini:
// 5V  -> +5V do Chicote Rinnai
// GND -> GND do Chicote Rinnai
// 4   -> RX do C3 (Ligar no TX da Central Rinnai)
// 5   -> TX do C3 (Ligar no RX da Central Rinnai)
#define RINNAI_RX_PIN  4
#define RINNAI_TX_PIN  5
#define LED_PIN        8  // LED azul integrado (ativo em LOW)

// ==========================================
// ESTRUTURA DE STATUS E CONTROLE RINNAI
// ==========================================
struct RinnaiData {
    volatile bool valid;
    volatile bool powerOn;
    volatile bool flameActive;
    volatile uint8_t temperature;
    volatile uint8_t rawPower;
    volatile uint8_t diagCode;
    volatile uint8_t rawBytes[6];
    volatile uint32_t lastPacketTime;
    volatile uint32_t packetCount;
};

RinnaiData rinnai = {false, false, false, 0, 0, 0, {0}, 0, 0};

// Temporizações do protocolo Rinnai (em microssegundos)
#define SYMBOL_PERIOD_MIN  450
#define SYMBOL_PERIOD_MAX  800
#define RESET_PULSE_MIN    900
#define SHORT_PULSE_MAX    300

volatile uint32_t lastFallTime = 0;
volatile uint32_t lastRiseTime = 0;
volatile uint8_t  bitIndex = 0;
volatile uint8_t  currentByte = 0;
volatile uint8_t  rxPacketBuffer[6];

// ==========================================
// ROTINA DE INTERRUPÇÃO (ISR) - DECODIFICADOR
// ==========================================
void IRAM_ATTR onRinnaiRxChange() {
    uint32_t now = micros();
    bool level = digitalRead(RINNAI_RX_PIN);

    if (level == HIGH) {
        lastRiseTime = now;
    } else {
        uint32_t highDuration = now - lastRiseTime;
        uint32_t totalPeriod  = now - lastFallTime;
        lastFallTime = now;

        // Pulso de Reset / Sincronismo inicial (> 900 us)
        if (highDuration >= RESET_PULSE_MIN || totalPeriod >= RESET_PULSE_MIN) {
            bitIndex = 0;
            currentByte = 0;
            return;
        }

        // Amostragem de símbolo (~600 us)
        if (totalPeriod >= SYMBOL_PERIOD_MIN && totalPeriod <= SYMBOL_PERIOD_MAX) {
            uint8_t bit = (highDuration > SHORT_PULSE_MAX) ? 1 : 0;
            currentByte |= (bit << (bitIndex % 8));
            bitIndex++;

            if ((bitIndex % 8) == 0) {
                uint8_t bytePos = (bitIndex / 8) - 1;
                if (bytePos < 6) {
                    rxPacketBuffer[bytePos] = currentByte;
                }
                currentByte = 0;

                if (bitIndex >= 48) {
                    // Validar Checksum XOR dos 5 primeiros bytes
                    uint8_t calcXor = rxPacketBuffer[0] ^ rxPacketBuffer[1] ^ rxPacketBuffer[2] ^ rxPacketBuffer[3] ^ rxPacketBuffer[4];
                    if (calcXor == rxPacketBuffer[5]) {
                        for (int i = 0; i < 6; i++) {
                            rinnai.rawBytes[i] = rxPacketBuffer[i];
                        }
                        rinnai.powerOn = (rxPacketBuffer[1] & 0x40) != 0;
                        rinnai.flameActive = (rxPacketBuffer[2] & 0x10) != 0;

                        uint8_t tempCode = rxPacketBuffer[2] & 0x0F;
                        if (tempCode == 0 && (rxPacketBuffer[1] & 0x08)) {
                            rinnai.temperature = 35;
                        } else if (tempCode <= 11) {
                            rinnai.temperature = 37 + tempCode;
                        } else if (tempCode == 12) {
                            rinnai.temperature = 50;
                        } else if (tempCode == 13) {
                            rinnai.temperature = 55;
                        } else if (tempCode == 14) {
                            rinnai.temperature = 60;
                        } else {
                            rinnai.temperature = 37 + tempCode;
                        }

                        rinnai.rawPower = rxPacketBuffer[3];
                        rinnai.diagCode = rxPacketBuffer[4];
                        rinnai.lastPacketTime = millis();
                        rinnai.packetCount++;
                        rinnai.valid = true;
                    }
                    bitIndex = 0;
                }
            }
        }
    }
}

// ==========================================
// TRANSMISSOR DE COMANDOS (ENCODER)
// ==========================================
void sendPulsePacket(const uint8_t pkt[6]) {
    // Desabilita interrupções temporariamente para precisão de timing
    noInterrupts();
    digitalWrite(RINNAI_TX_PIN, HIGH);
    delayMicroseconds(1200); // Reset pulse
    digitalWrite(RINNAI_TX_PIN, LOW);
    delayMicroseconds(400);

    for (int b = 0; b < 6; b++) {
        uint8_t val = pkt[b];
        for (int i = 0; i < 8; i++) {
            bool bit = (val >> i) & 1;
            uint32_t highTime = bit ? 450 : 150;
            uint32_t lowTime  = 600 - highTime;

            digitalWrite(RINNAI_TX_PIN, HIGH);
            delayMicroseconds(highTime);
            digitalWrite(RINNAI_TX_PIN, LOW);
            delayMicroseconds(lowTime);
        }
    }
    digitalWrite(RINNAI_TX_PIN, LOW);
    interrupts();
}

void pressRinnaiButton(const String& btn) {
    uint8_t pkt[6] = {0x00, 0x00, 0x00, 0x00, 0xBF, 0x00};

    if (btn == "up") {
        pkt[2] = 0x01; // Temp Up
    } else if (btn == "down") {
        pkt[2] = 0x02; // Temp Down
    } else if (btn == "power") {
        pkt[1] = 0x01; // Power Toggle
    }

    // Paridade ímpar no MSB de cada byte
    for (int i = 0; i < 5; i++) {
        int ones = 0;
        for (int bit = 0; bit < 7; bit++) {
            if ((pkt[i] >> bit) & 1) ones++;
        }
        if ((ones % 2) == 0) {
            pkt[i] |= 0x80;
        }
    }

    pkt[5] = pkt[0] ^ pkt[1] ^ pkt[2] ^ pkt[3] ^ pkt[4];
    sendPulsePacket(pkt);
}

// ==========================================
// SERVIDOR WEB
// ==========================================
WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Rinnai Remote Monitor</title>
  <style>
    :root {
      --bg: #0f172a;
      --card: #1e293b;
      --text: #f8fafc;
      --subtext: #94a3b8;
      --primary: #38bdf8;
      --flame-on: #f97316;
      --power-on: #22c55e;
      --border: #334155;
    }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
      background: var(--bg);
      color: var(--text);
      margin: 0;
      padding: 20px;
      display: flex;
      flex-direction: column;
      align-items: center;
    }
    .container {
      width: 100%;
      max-width: 480px;
    }
    .card {
      background: var(--card);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 24px;
      box-shadow: 0 10px 25px rgba(0,0,0,0.3);
      margin-bottom: 20px;
    }
    h1 {
      font-size: 1.5rem;
      margin: 0 0 16px 0;
      text-align: center;
      color: var(--primary);
    }
    .temp-display {
      text-align: center;
      margin: 20px 0;
    }
    .temp-val {
      font-size: 4.5rem;
      font-weight: 800;
      line-height: 1;
    }
    .temp-unit {
      font-size: 1.8rem;
      color: var(--subtext);
      vertical-align: super;
    }
    .status-badges {
      display: flex;
      justify-content: center;
      gap: 12px;
      margin-bottom: 24px;
    }
    .badge {
      padding: 6px 14px;
      border-radius: 9999px;
      font-size: 0.85rem;
      font-weight: 600;
      display: inline-flex;
      align-items: center;
      gap: 6px;
    }
    .badge-power-on { background: rgba(34, 197, 94, 0.2); color: #4ade80; border: 1px solid #22c55e; }
    .badge-power-off { background: rgba(148, 163, 184, 0.15); color: #94a3b8; border: 1px solid #475569; }
    .badge-flame-on { background: rgba(249, 115, 22, 0.2); color: #fb923c; border: 1px solid #f97316; }
    .badge-flame-off { background: rgba(148, 163, 184, 0.15); color: #94a3b8; border: 1px solid #475569; }
    .controls {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-bottom: 16px;
    }
    button {
      background: #0284c7;
      color: white;
      border: none;
      border-radius: 12px;
      padding: 16px;
      font-size: 1.2rem;
      font-weight: 700;
      cursor: pointer;
      transition: all 0.2s;
    }
    button:active {
      transform: scale(0.96);
      background: #0369a1;
    }
    .btn-power {
      grid-column: span 2;
      background: #334155;
    }
    .btn-power:active {
      background: #1e293b;
    }
    .raw-box {
      background: #0f172a;
      border: 1px solid var(--border);
      border-radius: 8px;
      padding: 12px;
      font-family: monospace;
      font-size: 0.85rem;
      color: #38bdf8;
      word-break: break-all;
    }
    .meta-row {
      display: flex;
      justify-content: space-between;
      color: var(--subtext);
      font-size: 0.8rem;
      margin-top: 12px;
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <h1>♨️ Rinnai Smart Remote</h1>
      
      <div class="status-badges">
        <span id="badge-power" class="badge badge-power-off">● STANDBY</span>
        <span id="badge-flame" class="badge badge-flame-off">🔥 CHAMA APAGADA</span>
      </div>

      <div class="temp-display">
        <span id="temp-val" class="temp-val">--</span><span class="temp-unit">°C</span>
      </div>

      <div class="controls">
        <button onclick="sendAction('down')">− TEMP</button>
        <button onclick="sendAction('up')">+ TEMP</button>
        <button class="btn-power" onclick="sendAction('power')">⏻ POWER LIGA / DESLIGA</button>
      </div>

      <div class="meta-row">
        <span>Diag: <b id="diag-code">--</b></span>
        <span>Pacotes: <b id="pkt-count">0</b></span>
      </div>
    </div>

    <div class="card">
      <div style="font-weight: 600; margin-bottom: 8px; font-size: 0.9rem;">Telemetria Bruta (RAW HEX):</div>
      <div id="raw-hex" class="raw-box">Aguardando dados...</div>
      <div class="meta-row">
        <span id="last-seen">Última atualização: --</span>
        <span id="wifi-rssi">Sinal: -- dBm</span>
      </div>
    </div>
  </div>

  <script>
    async function updateStatus() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();
        
        document.getElementById('temp-val').textContent = data.temperature > 0 ? data.temperature : '--';
        document.getElementById('diag-code').textContent = data.diagCodeHex;
        document.getElementById('pkt-count').textContent = data.packetCount;
        document.getElementById('raw-hex').textContent = data.rawHex;
        document.getElementById('wifi-rssi').textContent = 'WiFi: ' + data.rssi + ' dBm';
        document.getElementById('last-seen').textContent = 'Visto há ' + data.ageSeconds + 's';

        const bPower = document.getElementById('badge-power');
        if (data.powerOn) {
          bPower.textContent = '● LIGADO';
          bPower.className = 'badge badge-power-on';
        } else {
          bPower.textContent = '● STANDBY';
          bPower.className = 'badge badge-power-off';
        }

        const bFlame = document.getElementById('badge-flame');
        if (data.flameActive) {
          bFlame.textContent = '🔥 CHAMA ATIVA';
          bFlame.className = 'badge badge-flame-on';
        } else {
          bFlame.textContent = '○ CHAMA APAGADA';
          bFlame.className = 'badge badge-flame-off';
        }
      } catch (e) {
        console.error(e);
      }
    }

    async function sendAction(action) {
      try {
        await fetch('/api/button?action=' + action, { method: 'POST' });
        setTimeout(updateStatus, 300);
      } catch (e) {
        alert('Erro ao enviar comando: ' + e);
      }
    }

    setInterval(updateStatus, 1000);
    updateStatus();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleStatusApi() {
    uint32_t now = millis();
    uint32_t ageSec = (now - rinnai.lastPacketTime) / 1000;

    char rawHex[32];
    snprintf(rawHex, sizeof(rawHex), "%02X %02X %02X %02X %02X %02X",
             rinnai.rawBytes[0], rinnai.rawBytes[1], rinnai.rawBytes[2],
             rinnai.rawBytes[3], rinnai.rawBytes[4], rinnai.rawBytes[5]);

    char diagHex[8];
    snprintf(diagHex, sizeof(diagHex), "0x%02X", rinnai.diagCode);

    String json = "{";
    json += "\"valid\":" + String(rinnai.valid ? "true" : "false") + ",";
    json += "\"powerOn\":" + String(rinnai.powerOn ? "true" : "false") + ",";
    json += "\"flameActive\":" + String(rinnai.flameActive ? "true" : "false") + ",";
    json += "\"temperature\":" + String(rinnai.temperature) + ",";
    json += "\"diagCodeHex\":\"" + String(diagHex) + "\",";
    json += "\"rawHex\":\"" + String(rawHex) + "\",";
    json += "\"packetCount\":" + String(rinnai.packetCount) + ",";
    json += "\"ageSeconds\":" + String(ageSec) + ",";
    json += "\"rssi\":" + String(WiFi.RSSI());
    json += "}";

    server.send(200, "application/json", json);
}

void handleButtonApi() {
    if (server.hasArg("action")) {
        String action = server.arg("action");
        Serial.printf("[WEB] Ação solicitada: %s\n", action.c_str());
        pressRinnaiButton(action);
        server.send(200, "application/json", "{\"status\":\"ok\"}");
    } else {
        server.send(400, "application/json", "{\"error\":\"missing action\"}");
    }
}

// ==========================================
// SETUP & LOOP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH); // Apaga LED inicial (ativo em LOW)

    Serial.println("\n=========================================");
    Serial.println("  Rinnai Water Heater ESP32-C3 Firmware  ");
    Serial.println("=========================================");

    // Configuração dos pinos do barramento
    pinMode(RINNAI_RX_PIN, INPUT_PULLUP);
    pinMode(RINNAI_TX_PIN, OUTPUT);
    digitalWrite(RINNAI_TX_PIN, LOW);

    // Conectar ao Wi-Fi
    Serial.printf("[WIFI] Conectando a %s...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t tStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - tStart) < 15000) {
        delay(300);
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Pisca LED
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(LED_PIN, LOW); // LED aceso fixo = Conectado
        Serial.println("\n[WIFI] Conectado com sucesso!");
        Serial.print("[WIFI] Endereco IP: ");
        Serial.println(WiFi.localIP());

        // Iniciar mDNS (rinnai.local)
        if (MDNS.begin("rinnai")) {
            Serial.println("[MDNS] Disponivel em: http://rinnai.local/");
        }
    } else {
        Serial.println("\n[WIFI] Falha ao conectar. Iniciando em modo offline.");
    }

    // Configurar endpoints do servidor Web
    server.on("/", handleRoot);
    server.on("/api/status", handleStatusApi);
    server.on("/api/button", HTTP_POST, handleButtonApi);
    server.on("/api/button", HTTP_GET, handleButtonApi);
    server.begin();
    Serial.println("[HTTP] Servidor Web ativo na porta 80.");

    // Habilitar interrupção de leitura na porta RX (GPIO 4)
    attachInterrupt(digitalPinToInterrupt(RINNAI_RX_PIN), onRinnaiRxChange, CHANGE);
    Serial.printf("[RX] Interrupcao ativada no GPIO %d\n", RINNAI_RX_PIN);
    Serial.printf("[TX] Transmissao configurada no GPIO %d\n", RINNAI_TX_PIN);
}

void loop() {
    server.handleClient();

    // Log periódico via Serial a cada 2 segundos se houver pacote
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 2000) {
        lastLog = millis();
        if (rinnai.valid) {
            Serial.printf("[STATUS] Power: %s | Chama: %s | Temp: %d°C | Diag: 0x%02X | Pkts: %u\n",
                          rinnai.powerOn ? "LIGADO" : "STANDBY",
                          rinnai.flameActive ? "ATIVA" : "APAGADA",
                          rinnai.temperature,
                          rinnai.diagCode,
                          rinnai.packetCount);
        }
    }
}
