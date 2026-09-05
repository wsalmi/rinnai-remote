# Engenharia Reversa do Barramento Digital Rinnai & Controle Remoto ESP32

Este repositório documenta a engenharia reversa completa da camada física, temporização de sinais, codificação e estrutura de protocolo da comunicação entre a central eletrônica de aquecedores a gás **Rinnai** e seus painéis digitais (painel frontal integrado e controles remotos cabeados das linhas REU).

O projeto inclui documentação técnica aprofundada, ferramentas de análise em Python e firmware em C++ (PlatformIO / Arduino) pronto para implementação em microcontroladores ESP32.

---

## Sumário

1. [Visão Geral e Arquitetura do Repositório](#1-visão-geral-e-arquitetura-do-repositório)
2. [Camada Física e Interface Elétrica](#2-camada-física-e-interface-elétrica)
   - [Pinout do Chicote](#pinout-do-chicote)
   - [Condicionamento de Sinal (5V TTL vs. 3.3V CMOS)](#condicionamento-de-sinal-5v-ttl-vs-33v-cmos)
3. [Temporização de Sinais e Modulação de Pulso](#3-temporização-de-sinais-e-modulação-de-pulso)
   - [Por que NÃO é UART (8N1)? O Fenômeno dos 'U's (0x55)](#por-que-não-é-uart-8n1-o-fenômeno-dos-us-0x55)
   - [Modulação por Largura de Pulso (PWM / Pulse-Length)](#modulação-por-largura-de-pulso-pwm--pulse-length)
   - [Medições Reais de Hardware (Deltas de Pulso)](#medições-reais-de-hardware-deltas-de-pulso)
4. [Camada de Enlace e Formatação dos Quadros](#4-camada-de-enlace-e-formatação-dos-quadros)
   - [Estrutura do Pacote](#estrutura-do-pacote)
   - [Ordem dos Bits (LSB-First)](#ordem-dos-bits-lsb-first)
   - [Mecanismo de Paridade Ímpar (Odd Parity)](#mecanismo-de-paridade-ímpar-odd-parity)
   - [Mecanismo de Checksum XOR (LRC)](#mecanismo-de-checksum-xor-lrc)
5. [Mapeamento Completo de Bits do Payload](#5-mapeamento-completo-de-bits-do-payload)
   - [Pacote Central $\rightarrow$ Painel (Telemetria)](#pacote-central--painel-telemetria)
   - [Tabela Oficial de Conversão de Temperatura](#tabela-oficial-de-conversão-de-temperatura)
   - [Pacote Painel $\rightarrow$ Central (Injeção de Comandos)](#pacote-painel--central-injeção-de-comandos)
6. [Estudo de Caso Real: A Descoberta do Byte 0x15](#6-estudo-de-caso-real-a-descoberta-do-byte-0x15)
7. [Guia de Replicação para Especialistas](#7-guia-de-replicação-para-especialistas)
   - [Captura com Bus Pirate / Analisador Lógico](#captura-com-bus-pirate--analisador-lógico)
   - [Decodificação por Interrupção no ESP32](#decodificação-por-interrupção-no-esp32)
   - [Transmissão e Injeção de Comandos no TX](#transmissão-e-injeção-de-comandos-no-tx)
8. [Uso das Ferramentas Python](#8-uso-das-ferramentas-python)
9. [Compilação e Gravação do Firmware ESP32](#9-compilação-e-gravação-do-firmware-esp32)

---

## 1. Visão Geral e Arquitetura do Repositório

```text
rinnai-remote/
├── README.md                      # Esta especificação técnica completa
├── docs/                          # Documentação detalhada complementar
│   ├── physical_layer.md          # Níveis elétricos, divisores de tensão e osciladores
│   ├── protocol_spec.md           # Detalhes de quadros, paridade e comandos do painel
│   └── captures_and_analysis.md   # Logs reais brutos (Standby vs Chuveiro a 42°C)
├── tools/                         # Utilitários de captura e decodificação
│   ├── bp_capture.py              # Sniffer em tempo real via WebSocket do Bus Pirate
│   └── decoder.py                 # Biblioteca Python de validação, paridade e desempacotamento
└── firmware/                      # Código C++ para ESP32 (PlatformIO)
    ├── platformio.ini             # Configuração do ambiente esp32dev
    ├── include/
    │   ├── rinnai_decoder.h       # Decodificador não-bloqueante por interrupção GPIO
    │   └── rinnai_encoder.h       # Gerador de pulsos para injeção de botões (+, -, power)
    └── src/
        └── main.cpp               # Aplicação principal (Monitor Serial + comandos)
```

---

## 2. Camada Física e Interface Elétrica

### Pinout do Chicote
A central se comunica com o painel frontal digital através de um chicote com conector de 4 vias:

| Pino | Cor Típica | Sinal | Sentido | Nível Elétrico | Função |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | Vermelho | `+5V` | Central $\rightarrow$ Painel | $+5.0\,\text{V}$ DC | Alimentação positiva do painel frontal |
| **2** | Preto | `GND` | Bidirecional | $0\,\text{V}$ | Referência de terra comum |
| **3** | Amarelo / Branco | `TX` (Central) | Central $\rightarrow$ Painel | 5V TTL | Telemetria de status, temperatura e chama |
| **4** | Azul / Verde | `RX` (Central) | Painel $\rightarrow$ Central | 5V TTL | Envio de comandos dos botões físicos |

### Condicionamento de Sinal (5V TTL vs. 3.3V CMOS)

> [!CAUTION]
> Os pinos GPIO do ESP32 toleram no máximo **3.6V**. A linha de dados da central opera em **5V TTL**. Ligar a linha TX da central diretamente a um pino do ESP32 danificará o microcontrolador a médio/longo prazo.

Recomenda-se um divisor de tensão resistivo simples para a linha de recepção:

```text
Linha TX Central (5V) ─────[ 10 kΩ ]─────┬─────> GPIO 5 (RX ESP32 - 3.3V)
                                         │
                                      [ 20 kΩ ]
                                         │
                                        GND
```
$$V_{\text{out}} = 5\,\text{V} \times \frac{20\,\text{k}\Omega}{10\,\text{k}\Omega + 20\,\text{k}\Omega} \approx 3.33\,\text{V}$$

Para a linha de transmissão do ESP32 para a central (**GPIO 15 $\rightarrow$ RX Central**), o nível ALTO de 3.3V do ESP32 é superior ao limiar mínimo de nível alto ($V_{IH} = 2.0\,\text{V}$) da lógica TTL da central Rinnai, podendo ser conectado diretamente ou através de um resistor de proteção de $1\,\text{k}\Omega$.

---

## 3. Temporização de Sinais e Modulação de Pulso

### Por que NÃO é UART (8N1)? O Fenômeno dos 'U's (0x55)
Ao conectar um receptor UART convencional a **2800 baud** na linha de dados, obtém-se uma sequência repetitiva de caracteres:
```text
D\x0cQUUUUUEQUEDTTUEUEEQUUUEU\x0cEQUEDQUDEDQ5EUUUUTUUTQD...
```

Analisando a representação binária desses bytes:
* **`'U'` = `0x55` = `01010101`** (alternância perfeita de 0 e 1)
* **`'E'` = `0x45` = `01000101`**
* **`'Q'` = `0x51` = `01010001`**
* **`'T'` = `0x54` = `01010100`**

Isso ocorre devido ao **aliasing temporal** de amostragem:
1. A UART a 2800 baud possui período de bit $T_{\text{UART}} = \frac{1}{2800} \approx 357.1\,\mu\text{s}$.
2. O sinal da Rinnai possui um período de símbolo base de $\approx 370\,\mu\text{s}$.
3. O receptor UART interpreta cada descida de pulso como um *Start Bit*, realiza a leitura a cada $\approx 357\,\mu\text{s}$, capturando as subidas e descidas alternadas do PWM como se fossem bits `01010101` (`0x55` = `'U'`).

### Modulação por Largura de Pulso (PWM / Pulse-Length)
O protocolo da Rinnai codifica a informação binária através da **relação de tempo em nível ALTO / BAIXO**:

```text
               Período de Símbolo: ~600 µs a ~740 µs
               
Bit 0:        ┌──────┐
(Curto ~25%)  │150 µs│                  450 µs
       ───────┘      └────────────────────────────────────────
       
Bit 1:        ┌────────────────────────────┐
(Longo ~75%)  │           450 µs           │      150 µs
       ───────┘                            └──────────────────
       
Reset / Sync: ┌──────────────────────────────────────────────┐ (> 900 µs)
(Pausa)       │                                              │
       ───────┘                                              └───
```

### Medições Reais de Hardware (Deltas de Pulso)
Medições obtidas via analisador digital de transição de bordas (`sniff` de hardware no pino 5):
* **$\Delta t \approx 368\,\mu\text{s} \dots 370\,\mu\text{s}$**: Duração elementar de 1 bit base ($1\times$).
* **$\Delta t \approx 1108\,\mu\text{s}$**: Duração de 3 tempos base ($3 \times 369.3\,\mu\text{s}$).
* **$\Delta t \approx 1847\,\mu\text{s}$**: Duração de 5 tempos base ($5 \times 369.4\,\mu\text{s}$).
* **$\Delta t \approx 7200\,\mu\text{s}$**: Silêncio em repouso entre pacotes (*Inter-frame gap*).

Base de tempo fundamental do oscilador da central:
$$f_{\text{base}} = \frac{1}{370 \times 10^{-6}\,\text{s}} \approx \mathbf{2702\text{ bps}}$$

---

## 4. Camada de Enlace e Formatação dos Quadros

### Estrutura do Pacote
Cada quadro é constituído por:
* **1 Pulso de Sincronismo / Reset inicial:** Duração superior a $900\,\mu\text{s}$.
* **48 Símbolos de Dados:** 48 pulsos modulados = **6 Bytes exatos**.
* **Intervalo de Repouso:** 7 a 20 ms de linha ociosa antes do próximo ciclo.
* **Taxa de Repetição:** O pacote completo é retransmitido a cada **150 ms a 200 ms**.

### Ordem dos Bits (LSB-First)
A transmissão de cada byte ocorre sempre iniciando pelo bit menos significativo:
$$\text{Ordem no Barramento: } b_0 \rightarrow b_1 \rightarrow b_2 \rightarrow b_3 \rightarrow b_4 \rightarrow b_5 \rightarrow b_6 \rightarrow b_7 \ (\text{MSB})$$

### Mecanismo de Paridade Ímpar (Odd Parity)
Cada um dos 6 bytes utiliza o **Bit 7 (MSB)** como bit de paridade ímpar:

$$\text{Bit 7} = \begin{cases} 
0, & \text{se } \sum_{i=0}^{6} b_i \text{ for ÍMPAR} \\ 
1, & \text{se } \sum_{i=0}^{6} b_i \text{ for PAR} 
\end{cases}$$

Dessa forma, o total de bits `1` presentes no byte (incluindo o bit 7) **é sempre ímpar**:
$$\left( \sum_{i=0}^{7} b_i \right) \equiv 1 \pmod 2$$

#### Algoritmo de Validação de Paridade em C++:
```cpp
bool checkOddParity(uint8_t byteVal) {
    uint8_t count = 0;
    for (int i = 0; i < 8; i++) {
        if ((byteVal >> i) & 1) count++;
    }
    return (count % 2) != 0; // Verdadeiro se ímpar
}
```

#### Algoritmo de Geração de Paridade para Envio em C++:
```cpp
uint8_t addOddParity(uint8_t data7bit) {
    uint8_t count = 0;
    for (int i = 0; i < 7; i++) {
        if ((data7bit >> i) & 1) count++;
    }
    // Se a contagem for par, seta o bit 7 em 1 para tornar ímpar
    if ((count % 2) == 0) {
        return data7bit | 0x80;
    }
    return data7bit & 0x7F;
}
```

### Mecanismo de Checksum XOR (LRC)
A integridade global do pacote é garantida pelo 6º byte ($\text{Byte}[5]$), que consiste na operação XOR acumulada dos 5 primeiros bytes:

$$\text{Byte}[5] = \text{Byte}[0] \oplus \text{Byte}[1] \oplus \text{Byte}[2] \oplus \text{Byte}[3] \oplus \text{Byte}[4]$$

Se a soma calculada for idêntica ao $\text{Byte}[5]$ recebido, o quadro é considerado íntegro e validado para processamento.

---

## 5. Mapeamento Completo de Bits do Payload

### Pacote Central $\rightarrow$ Painel (Telemetria)

Formato dos 6 bytes emitidos pela central no pino **TX (Central) / RX (ESP32)**:

```text
┌──────────────┬──────────────┬──────────────┬──────────────┬──────────────┬──────────────┐
│    Byte 0    │    Byte 1    │    Byte 2    │    Byte 3    │    Byte 4    │    Byte 5    │
├──────────────┼──────────────┼──────────────┼──────────────┼──────────────┼──────────────┤
│  Header/ID   │ Status Sist. │ Chama & Temp │  Potência/Ex │ Diag/Erros   │ Checksum XOR │
└──────────────┴──────────────┴──────────────┴──────────────┴──────────────┴──────────────┘
```

#### Detalhamento Campo a Campo:

* **Byte 0 (Cabeçalho e Prioridade):**
  * `Bits 0..3`: Fixo em `0x7` (indica identificador de origem: Central Principal).
  * `Bits 4..6`: Prioridade da central no barramento.
  * `Bit 7`: Paridade ímpar.
* **Byte 1 (Estado de Energia e Escala):**
  * `Bit 6 (0x40)`: **Sistema Ligado / Standby** (`1` = Ligado, `0` = Standby/Desligado).
  * `Bit 3 (0x08)`: **Escala Secundária** (habilitado quando a temperatura é ajustada para 35°C).
  * `Bit 7`: Paridade ímpar.
* **Byte 2 (Chama e Ajuste de Temperatura):**
  * `Bits 0..3`: **Código Linear de Temperatura** (ver tabela a seguir).
  * `Bit 4 (0x10)`: **Chama Ativa / Queimador em Uso** (`1` = Fogo aceso, fluxo de água detectado).
  * `Bit 7`: Paridade ímpar.
* **Byte 3 (Modulação da Válvula de Gás e Exaustor):**
  * Bits de comando da modulação proporcional de potência (Low, Medium, High).
  * Feedback do tacômetro do ventilador de exaustão forçada.
* **Byte 4 (Diagnóstico e Falhas):**
  * `0x20`: Operação normal sem erros.
  * Outros valores: Código de erro ativo mostrado no visor (Ex.: erro `11` = falha de acendimento, `12` = perda de chama por falta de gás, `90` = exaustão obstruída).
* **Byte 5 (Checksum):**
  * $\text{Byte}[0] \oplus \text{Byte}[1] \oplus \text{Byte}[2] \oplus \text{Byte}[3] \oplus \text{Byte}[4]$.

---

### Tabela Oficial de Conversão de Temperatura

A temperatura programada é decodificada a partir dos bits `[3:0]` do Byte 2:
$$\text{Temperatura (°C)} = 37 + \text{Código}$$

| Código Hex (`Byte 2 [3:0]`) | Flag Byte 1 (`0x08`) | Temperatura Resultante | Comportamento no Visor |
| :---: | :---: | :---: | :--- |
| `0x0` | **Ativo (1)** | **35°C** | Modo temperatura mínima de verão |
| `0x0` | Inativo (0) | **37°C** | Base linear de aquecimento |
| `0x1` | Inativo (0) | **38°C** | Incremento +1°C |
| `0x2` | Inativo (0) | **39°C** | Incremento +1°C |
| `0x3` | Inativo (0) | **40°C** | Incremento +1°C |
| `0x4` | Inativo (0) | **41°C** | Incremento +1°C |
| **`0x5`** | Inativo (0) | **42°C** | **Confirmado em laboratório (Byte 2 = `0x15`)** |
| `0x6` | Inativo (0) | **43°C** | Incremento +1°C |
| `0x7` | Inativo (0) | **44°C** | Incremento +1°C |
| `0x8` | Inativo (0) | **45°C** | Incremento +1°C |
| `0x9` | Inativo (0) | **46°C** | Incremento +1°C |
| `0xA` | Inativo (0) | **47°C** | Incremento +1°C |
| `0xB` | Inativo (0) | **48°C** | Incremento +1°C |
| `0xC` | Inativo (0) | **50°C** | Salto de escala (não há 49°C) |
| `0xD` | Inativo (0) | **55°C** | Salto de escala |
| `0xE` | Inativo (0) | **60°C** | Temperatura máxima padrão |

---

### Pacote Painel $\rightarrow$ Central (Injeção de Comandos)

Formato transmitido pelo painel frontal ou pelo ESP32 no pino **RX (Central) / TX (ESP32)** para simular o pressionamento dos botões:

| Byte | Campo | Mapeamento de Bits |
| :---: | :--- | :--- |
| **0** | **ID do Painel** | `[7:4] = 0x0` (origem: painel frontal). `[3:0]` ID de estação. |
| **1** | **Botões de Sistema** | `Bit 0 (0x01)`: Botão **Power (Liga/Desliga)** pressionado.<br>`Bit 2 (0x04)`: Botão **Prioridade** pressionado. |
| **2** | **Botões de Temperatura** | `Bit 0 (0x01)`: Botão **Temperatura + (Up)** pressionado.<br>`Bit 1 (0x02)`: Botão **Temperatura - (Down)** pressionado. |
| **3** | **Handshake** | Estado do barramento. |
| **4** | **Constante** | Fixo em `0xBF`. |
| **5** | **Checksum** | XOR dos Bytes 0 a 4. |

---

## 6. Estudo de Caso Real: A Descoberta do Byte 0x15

Durante os testes em bancada, comparamos o tráfego com o aquecedor em repouso e com o chuveiro em funcionamento a 42°C:

### Quadro em Standby (Repouso)
```hex
51 55 55 55 55 55 45 51 55 45 44 54 54 55 45 55 45 45 51 55 55 55 45 55 0C ...
```

### Quadro com Chuveiro Ligado a 42°C
```hex
51 15 55 55 45 55 55 15 54 55 15 44 51 15 45 00 00 55 14 44 51 15 51 15 51 ...
```

### Desmontagem Analítica do Byte `0x15`:
O byte **`0x15`** surgiu na transmissão com a abertura do fluxo de água:

$$\mathbf{0x15} = \mathbf{0001} \ \mathbf{0101}_b$$

1. **`Bit 4 = 1` (`0x10`):** Flag de **Chama Ativa / Burner In Use**. Acionada quando o sensor de fluxo aciona a ignição e a ionização confirma a chama.
2. **`Bits 0..3 = 0101_b` (`0x05`):** Código de temperatura:
   $$\text{Temperatura} = 37 + 5 = \mathbf{42^\circ\text{C}}$$
3. **`Bit 7 = 0`:** Paridade ímpar: o número de bits 1 em `0001 0101` é exatamente 2 (par), logo o bit 7 assume `0` para que a paridade seja mantida.

---

## 7. Guia de Replicação para Especialistas

### Captura com Bus Pirate / Analisador Lógico
Para verificar o sinal em hardware antes de compilar o firmware:
1. No terminal do Bus Pirate (Bruce Firmware), selecione o modo **DIO** (`m` $\rightarrow$ `9`).
2. Execute o analisador lógico com amostragem gráfica na tela do dispositivo:
   ```text
   logic 5
   ```
3. Ou utilize o rastreador de tempos de transição:
   ```text
   sniff 5
   ```
   Você deverá observar os deltas característicos de $\sim 368\,\mu\text{s}$ e $\sim 1108\,\mu\text{s}$.

### Decodificação por Interrupção no ESP32
O arquivo [rinnai_decoder.h](file:///Users/wesleysalmi/source-me/rinnai-remote/firmware/include/rinnai_decoder.h) implementa uma máquina de estados de interrupção não bloqueante (`IRAM_ATTR`):

1. A cada transição (`CHANGE`) no pino RX, a rotina calcula $\Delta t = \text{now} - \text{lastRiseTime}$.
2. Se $\Delta t > 900\,\mu\text{s}$, a rotina identifica o pulso de **Reset** e sincroniza o índice de bits em zero.
3. Se a duração estiver entre $450\,\mu\text{s}$ e $800\,\mu\text{s}$, avalia se o tempo em ALTO é superior a $300\,\mu\text{s}$ (Bit 1) ou inferior (Bit 0).
4. Ao acumular 48 bits, confere o Checksum XOR e disponibiliza o objeto estruturado `RinnaiStatus`.

### Transmissão e Injeção de Comandos no TX
O arquivo [rinnai_encoder.h](file:///Users/wesleysalmi/source-me/rinnai-remote/firmware/include/rinnai_encoder.h) gera os pulsos com precisão de microssegundos no pino TX (GPIO 15):
* Reset inicial: $1200\,\mu\text{s}$ em HIGH.
* Bit 0: $150\,\mu\text{s}$ em HIGH e $450\,\mu\text{s}$ em LOW.
* Bit 1: $450\,\mu\text{s}$ em HIGH e $150\,\mu\text{s}$ em LOW.

---

## 8. Uso das Ferramentas Python

### A. Teste Unitário do Decodificador:
```bash
cd tools
python3 decoder.py
```
*Saída esperada:*
```text
Teste de decodificação:
  source: MAIN
  power_on: True
  flame_active: True
  temperature_celsius: 42
  temperature_code: 5
  diag_code_hex: 0x20
  checksum_valid: True
```

### B. Sniffer em Tempo Real Conectado ao Bus Pirate:
```bash
cd tools
python3 bp_capture.py --host 192.168.68.112
```
O script estabelece um WebSocket direto com o Bus Pirate, ativa o modo bridge e analisa os quadros recebidos em tempo real.

---

## 9. Compilação e Gravação do Firmware ESP32

Requisitos: [PlatformIO Core (CLI)](https://docs.platformio.org/en/latest/core/installation/index.html) ou extensão PlatformIO no VS Code.

```bash
cd firmware

# Compilar o código
pio run

# Gravar no ESP32 conectado via USB
pio run --target upload

# Abrir o monitor serial
pio device monitor -b 115200
```

### Comandos Interativos no Monitor Serial:
* Pressione `+` para enviar pulso de aumento de temperatura.
* Pressione `-` para enviar pulso de diminuição de temperatura.
* Pressione `p` para enviar pulso de Liga/Desliga (Power Toggle).
