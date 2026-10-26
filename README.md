# Rinnai Remote

Engenharia reversa do barramento digital entre a central eletrônica de aquecedores a gás **Rinnai** e o painel frontal (e controles remotos cabeados da linha REU). O barramento não é UART 8N1: a informação vai em pulsos de largura variável. Este repositório guarda a documentação dessa camada física e do enquadramento, ferramentas em Python para captura e decodificação, e firmware para ESP32 que lê a telemetria e injeta cliques de botão.

A especificação longa está em [`docs/`](docs/). Este arquivo descreve o que o código faz hoje e como executá-lo.

## O que está incluído

- Notas de camada física, quadros e capturas de bancada em `docs/`.
- Decodificador Python de pacotes de 6 bytes (`tools/decoder.py`), com teste embutido, decodificação de um pacote e monitor contínuo (simulação, pipe ou WebSocket).
- Scripts que falam com um Bus Pirate no firmware Bruce, em parte dos comentários associado a um M5StickC Plus2, pela serial USB ou por WebSocket.
- Painel de terminal (`tools/live_dashboard.py`) que amostra a linha como UART a 2800 baud e interpreta quadros de 63 bytes.
- Coletores interativos que gravam amostras de temperatura e de ligado/desligado em JSON.
- Firmware PlatformIO para ESP32 (`firmware/`): monitor serial e comandos `+`, `-` e `p`.
- Sketch Arduino para ESP32-C3 Super Mini (`firmware_c3/`): mesma decodificação, mais interface web na porta 80.

Há dois caminhos de decodificação no repositório, e eles não são o mesmo algoritmo:

| Caminho | Onde | O que interpreta |
| :--- | :--- | :--- |
| Pacote de 6 bytes | `firmware/`, `firmware_c3/`, `tools/decoder.py`, `tools/bp_capture.py` | 48 bits após o pulso de sincronismo, paridade ímpar e checksum XOR |
| Amostra UART de 63 bytes | `tools/live_dashboard.py`, `tools/bp_temp_sniffer.py`, coletores | Fluxo 8N1 a 2800 baud, alinhado pelo preâmbulo `15 45 15 51` |

## Stack

- Python 3, sem `requirements.txt`. Os scripts de serial importam PySerial (`import serial`).
- PlatformIO, plataforma `espressif32`, placa `esp32dev`, framework Arduino (`firmware/platformio.ini`).
- Sketch Arduino para ESP32-C3, usando as bibliotecas de placa `WiFi`, `WebServer` e `ESPmDNS` (`firmware_c3/firmware_c3.ino`). Não há `platformio.ini` nesse diretório.
- Hardware citado no código e nas notas: central Rinnai, Bus Pirate (firmware Bruce) e ESP32 ou ESP32-C3 Super Mini (HW-466AB).

## Requisitos

- Python 3 no `PATH`. O teste do decodificador foi executado com Python 3.12.3.
- PySerial, apenas para os scripts que abrem a porta serial. Neste ambiente o módulo `serial` não está instalado; `tools/decoder.py` e `tools/bp_capture.py --help` não dependem dele.
- PlatformIO Core (ou a extensão no editor) para compilar `firmware/`. O comando `pio` não está instalado neste ambiente, então a compilação não foi executada aqui.
- Placa e core Arduino-ESP32 para gravar `firmware_c3/`.
- Para as ferramentas de captura: Bus Pirate acessível por serial USB (115200 baud na console) ou por WebSocket na rede. Os coletores e o painel pedem interação no painel físico do aquecedor.

## Configuração

Não há arquivo `.env`. Os nomes abaixo são constantes ou opções de linha de comando. Os valores que já estão no código (rede Wi-Fi, caminho de serial, IP padrão) não são repetidos aqui.

| Nome | Onde | Função |
| :--- | :--- | :--- |
| `WIFI_SSID`, `WIFI_PASS` | `firmware_c3/firmware_c3.ino` | Rede Wi-Fi em modo estação. Edite os dois antes de gravar a placa. |
| `RINNAI_RX_PIN`, `RINNAI_TX_PIN` | `firmware/src/main.cpp` | GPIO 5 (recepção) e GPIO 15 (transmissão) |
| `RINNAI_RX_PIN`, `RINNAI_TX_PIN`, `LED_PIN` | `firmware_c3/firmware_c3.ino` | GPIO 4 (RX), GPIO 5 (TX) e GPIO 8 (LED da placa, ativo em nível baixo) |
| `PORT`, `BAUD` | scripts em `tools/` que usam serial | Caminho do Bus Pirate e baud da console (`115200`). O padrão no código é um device macOS `cu.usbserial`; em Linux ajuste para o device local. |
| `--host`, `--port` | `tools/decoder.py`, `tools/bp_capture.py` | IP e porta HTTP/WebSocket do Bus Pirate. `--port` padrão: 80. |
| `--port`, `--pin`, `--sniff-duration` | `tools/bp_serial_sniff.py` | Serial, pino (`0`, `25` ou `26`) e duração do `sniff` (padrão 10 s) |
| `-w` / `--watch` / `--live`, `--demo`, `-i` / `--interval`, `--no-color` | `tools/decoder.py` | Monitor contínuo. `--interval` padrão: 0,2 s. O argumento `--demo` existe no `--help`, mas o programa não consulta esse flag: a simulação é o caminho padrão de `--watch` quando não há `--host` e o stdin é um terminal. |

O sketch C3 anuncia mDNS com o nome `rinnai` (`http://rinnai.local/` quando o resolvedor local funciona). A espera de associação Wi-Fi é de 15 s; se falhar, o servidor HTTP ainda sobe.

Scripts que fixam `PORT` e precisam ser editados para outra máquina:

- `tools/bp_grove_monitor.py`
- `tools/bp_live_monitor.py`
- `tools/bp_serial_sniff.py` (o padrão de `--port`)
- `tools/bp_temp_sniffer.py`
- `tools/interactive_collector.py`
- `tools/live_dashboard.py`
- `tools/power_state_collector.py`

A UART passiva configurada por esses scripts no Bus Pirate usa RX no pino 25, TX dummy no pino 10, 2800 baud, 8N1, e em seguida o comando `bridge`.

## Como executar

Comandos abaixo partem da raiz do repositório. Os de Python do decodificador foram executados aqui. Os demais dependem de hardware ou de ferramentas que não estão instaladas neste ambiente.

### Decodificador (executado)

Teste embutido, sem hardware:

```bash
python3 tools/decoder.py
```

Saída obtida:

```text
Teste de decodificação:
  source: MAIN
  power_on: True
  flame_active: True
  temperature_celsius: 42
  temperature_code: 5
  power_exhaust_raw: 0
  diag_code_hex: 0x20
  has_error: False
  checksum_valid: True
  parity_valid: False
  raw_hex: 57 40 15 00 20 22

Dica: Para monitoramento contínuo em tempo real, use: python3 decoder.py --watch
```

`parity_valid: False` é o resultado desse pacote de teste: nem todo byte tem paridade ímpar. O checksum XOR confere.

Decodificar seis bytes informados na linha de comando:

```bash
python3 tools/decoder.py 57 40 15 00 20 22
```

Monitor contínuo. Em um terminal, sem `--host`, o programa gera pacotes simulados. Com stdin redirecionado, lê linhas hexadecimais. Com `--host`, abre WebSocket em `/ws` e envia `bridge`.

```bash
python3 tools/decoder.py --watch --no-color
python3 tools/decoder.py --watch --host <ip-do-bus-pirate> --port 80
```

Ajuda: `python3 tools/decoder.py --help` e `python3 tools/bp_capture.py --help`.

Sniffer WebSocket (o padrão de `--host` está no código; passe o IP do aparelho):

```bash
python3 tools/bp_capture.py --host <ip-do-bus-pirate> --port 80
```

### Ferramentas de serial

Exigem PySerial e o Bus Pirate na porta configurada em `PORT`. Não foram executadas aqui (não há dispositivo serial nem o módulo `serial`).

| Comando | O que faz |
| :--- | :--- |
| `./run_dashboard.sh` | Sobe `tools/live_dashboard.py`. Prefere `~/.platformio/penv/bin/python3` e cai para `python3`. Pode ser chamado de outro diretório. |
| `./run_collector.sh` | Sobe `tools/interactive_collector.py` (35 °C a 46 °C, grava `tools/captured_temps.json`). Mesma escolha de interpretador. |
| `./run_power_collector.sh` | Chama `~/.platformio/penv/bin/python3 tools/power_state_collector.py` sem fallback. O caminho é relativo: rode da raiz do repositório. Temperaturas pedidas: 38, 42, 47, 48, 50 e 60 °C. Grava `tools/power_states.json`. |
| `python3 tools/bp_temp_sniffer.py` | UART a 2800 baud no pino 25 e imprime a temperatura calculada por `live_dashboard.calculate_temperature`. |
| `python3 tools/bp_live_monitor.py` | Mede G25 e G26 em modo DIO e dispara `sniff` quando há atividade. |
| `python3 tools/bp_grove_monitor.py` | O mesmo para a porta Grove, pinos G32 e G33. |
| `python3 tools/bp_serial_sniff.py --pin 25 --sniff-duration 10` | Lê estado e frequência dos pinos 0, 25 e 26 e faz `sniff` no pino escolhido. |

O painel e os coletores são interativos: ajustam o aquecedor no painel físico e, nos coletores, confirmam cada passo com Enter (`s` pula, `q` encerra).

### Firmware ESP32 (PlatformIO)

Definido em `firmware/platformio.ini` (`monitor_speed` 115200). Com o [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/index.html) instalado:

```bash
cd firmware
pio run
pio run --target upload
pio device monitor
```

No monitor serial, o sketch aceita:

- `+` — temperatura para cima
- `-` — temperatura para baixo
- `p` ou `P` — liga/desliga

Esses comandos estão em `firmware/src/main.cpp`. A placa não foi gravada neste ambiente.

### Firmware ESP32-C3

Abra `firmware_c3/firmware_c3.ino` no ambiente Arduino da placa ESP32-C3 Super Mini, ajuste `WIFI_SSID` e `WIFI_PASS`, e grave pela USB. Não há script de build no repositório.

Com Wi-Fi associado, a interface fica em `http://<ip-da-placa>/` e, se o mDNS responder, em `http://rinnai.local/`. Endpoints implementados:

- `GET /` — página com temperatura, estado, chama, diagnóstico, contagem de pacotes e hex bruto
- `GET /api/status` — JSON desses campos, mais idade do último pacote e RSSI
- `GET` ou `POST /api/button?action=up|down|power` — injeta o clique correspondente

A página consulta `/api/status` a cada 1 s.

## Protocolo, em resumo

O chicote documentado em [`docs/physical_layer.md`](docs/physical_layer.md) tem quatro vias: +5 V, GND, telemetria da central para o painel (5 V TTL) e comandos do painel para a central (5 V TTL). Os GPIO do ESP32 não são tolerantes a 5 V. A nota recomenda divisor 10 kΩ / 20 kΩ na recepção (no firmware PlatformIO, GPIO 5). O nível alto de 3,3 V na transmissão (GPIO 15 nesse firmware) é tratado na mesma nota como suficiente para o limiar TTL da central. O sketch C3 comenta a ligação direta: GPIO 4 no TX da central e GPIO 5 no RX da central.

Uma UART a cerca de 2800 baud enxerga o sinal como uma sequência cheia de `U` (`0x55`). [`docs/physical_layer.md`](docs/physical_layer.md) registra deltas de borda próximos de 370 µs, 1108 µs, 1847 µs e 7200 µs. O encoder em `firmware/include/rinnai_encoder.h` usa outro conjunto de constantes: período de 600 µs, bit 0 com 150 µs em alto, bit 1 com 450 µs em alto, e pulso de reset de 1200 µs. O decodificador aceita período de símbolo entre 450 µs e 800 µs, trata duração em alto acima de 300 µs como bit 1, e ressincroniza quando o pulso passa de 900 µs.

No modelo de 6 bytes usado pelo firmware e por `tools/decoder.py`:

- 6 bytes, bit menos significativo primeiro.
- O bit 7 de cada byte é paridade ímpar.
- O byte 5 é o XOR dos bytes 0 a 4.
- Telemetria: bit 6 do byte 1 indica ligado; bit 3 do byte 1 entra na temperatura de 35 °C; bit 4 do byte 2 indica chama; os bits 0–3 do byte 2 são o código de temperatura; o byte 4 é diagnóstico (`0x20` no código significa sem erro).
- Temperatura implementada: código 0 com o bit `0x08` do byte 1 → 35 °C; códigos 0–11 → 37 °C + código; código 12 → 50 °C; 13 → 55 °C; 14 → 60 °C.
- Comando de botão: byte 4 fixo em `0xBF`; power no bit 0 do byte 1; temperatura `+` no bit 0 do byte 2; temperatura `-` no bit 1 do byte 2. O encoder calcula paridade e checksum antes de transmitir.

O mapeamento campo a campo, a tabela de temperatura e o pacote de comando estão em [`docs/protocol_spec.md`](docs/protocol_spec.md). Comparações de captura (incluindo chama apagada e acesa) estão em [`docs/captures_and_analysis.md`](docs/captures_and_analysis.md).

O painel Python não usa esse pacote de 6 bytes. Ele procura um quadro de 63 bytes, estima chama pela contagem de bytes `0x77` e calcula a temperatura em `calculate_temperature`.

## Estrutura

```text
├── README.md
├── LICENSE
├── docs/
│   ├── physical_layer.md
│   ├── protocol_spec.md
│   └── captures_and_analysis.md
├── firmware/                      # PlatformIO, ESP32
│   ├── platformio.ini
│   ├── include/rinnai_decoder.h
│   ├── include/rinnai_encoder.h
│   └── src/main.cpp
├── firmware_c3/
│   └── firmware_c3.ino            # ESP32-C3, Wi-Fi e HTTP
├── run_collector.sh
├── run_dashboard.sh
├── run_power_collector.sh
└── tools/
    ├── decoder.py
    ├── bp_capture.py              # WebSocket
    ├── bp_serial_sniff.py
    ├── bp_live_monitor.py         # G25 e G26
    ├── bp_grove_monitor.py        # G32 e G33
    ├── bp_temp_sniffer.py
    ├── live_dashboard.py
    ├── interactive_collector.py
    ├── power_state_collector.py
    ├── captured_temps.json
    ├── captured_39.json
    ├── captured_40.json
    └── power_states.json
```

`.gitignore` ignora `__pycache__/`, `*.pyc`, `.DS_Store`, `.pio/` e `.vscode/`.

## Licença

MIT. Copyright (c) 2026 Wesley Salmi. Texto completo em [`LICENSE`](LICENSE).
