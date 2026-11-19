# Guia de Montagem Elétrica: M5NanoC6 + 2 Placas Optoacopladoras PC817 (`HY-M154` + Módulo 2 Canais) + Zigbee2MQTT / HomeKit

Este guia documenta a arquitetura definitiva utilizada em produção para monitorar e controlar o aquecedor a gás **Rinnai** utilizando um **M5Stack M5NanoC6** (ESP32-C6 com Zigbee nativo), integrando-o ao **Zigbee2MQTT** e **Homebridge** (Apple HomeKit) usando a porta **Grove** (`HY2.0-4P`) e **duas placas optoacopladoras `PC817` separadas**: **1 canal do módulo `HY-M154` (4 canais)** para o botão **`+`** e **1 canal do módulo isolador `PC817` de 2 canais (`3,6V–30V`)** para o botão **`−`**.

---

## 1. Por que abandonamos a injeção digital no pino RX?

Nos testes anteriores, ao conectar a saída do microcontrolador no fio `RX` da central (onde o painel frontal original envia os quadros de botões), ocorria **colisão elétrica no barramento (*bus contention*)**:

- O painel original da Rinnai mantém a linha `RX` ativamente controlada para transmitir quadros contínuos de status/handshake.
- Quando o ESP32 colocava seu pino em modo saída (*Push-Pull*) e tentava injetar pulsos, os dois transmissores entravam em conflito de nível lógico na mesma linha, corrompendo o quadro de sincronismo e fazendo o painel frontal entrar em estado de erro/pânico.

### A Solução: Leitura Passiva + Atuação Isolada nos Botões (`+` e `-`)
1. **Telemetria 100% Passiva (`G1`):** O M5NanoC6 apenas **escuta** o fio `TX` da central através de um divisor de tensão resistivo (alta impedância, sem jamais transmitir nada no barramento).
2. **Atuação via Contato Seco Optoisolado (`G2` + 2 Placas PC817):** Em vez de gerar pacotes no fio `RX`, fechamos eletronicamente os contatos dos botões físicos **`+`** e **`−`** do painel frontal usando dois canais `PC817` em placas separadas. Assim, o microcontrolador original do próprio painel gera os quadros válidos, mantém 100% das travas de segurança originais da Rinnai e preserva o bip sonoro nativo.

---

## 2. Mapeamento do M5NanoC6 e Conector Grove

O M5NanoC6 possui um conector Grove (`HY2.0-4P`) com apenas **2 GPIOs** expostos (`G1` e `G2`), além de `+5V` e `GND`:

| Pino Grove (Cor Padrão) | Pino M5NanoC6 | Função no Projeto |
| :--- | :--- | :--- |
| **Preto** | `GND` | Terra comum com a central Rinnai (`Fio 2 - Preto`) |
| **Vermelho** | `5V` | Alimentação $+5\,\text{V}$ vinda do chicote da central (`Fio 1 - Vermelho`) |
| **Branco** | **`G1` (`GPIO 1`)** | **Leitura Passiva via RMT RX em hardware ($1\,\mu\text{s}$/tick, 64 bits + CRC-16 `0xE815`)** ligada ao `TX` da central via divisor $10\,\text{k}\Omega / 20\,\text{k}\Omega$ |
| **Amarelo** | **`G2` (`GPIO 2`)** | **Saída Tri-State** emendada em "Y" para acionar as 2 placas optoacopladoras (`HY-M154` para `+` e `PC817 2 Canais` para `−`) |
| *(Não ligado)* | — | **`Fio 4` (`RX Central / TX Painel`)** não é ligado ao M5NanoC6; segue apenas entre central e painel |

### Periféricos Integrados Utilizados no M5NanoC6
- **`G9` (Botão físico):** Segurar por **5 segundos** executa Factory Reset do Zigbee e entra em modo de pareamento (ou segurar por $\ge 1.5\,\text{s}$ no boot limpa a NVRAM Zigbee).
- **`G19` + `G20` (LED RGB WS2812):** `G19` habilita a alimentação do LED; `G20` mostra o status:
  - **Azul piscando:** Aguardando pareamento / conexão Zigbee
  - **Verde fraco:** Conectado ao Zigbee, telemetria OK (chama apagada)
  - **Laranja:** Chama ativa (aquecendo água no banho)
  - **Âmbar fraco:** Conectado ao Zigbee, sem sinal recente da central
  - **Piscada rápida Ciano / Magenta:** Simulando clique no botão `+` ou `−`
  - **Roxo piscando / Vermelho sólido:** Botão `G9` pressionado para reset/pareamento Zigbee
- **`G7` (LED Azul discreto):** Pisca rapidamente a cada rajada de telemetria decodificada da central.

---

## 3. O Circuito Tri-State no Pino `G2` com 2 Placas PC817 Separadas

Como temos apenas **1 pino livre (`G2`)** para acionar **2 botões (`+` e `−`)**, utilizamos a técnica de **Lógica Tri-State (3 estados)** aproveitando a queda de tensão dos LEDs internos dos módulos `PC817`.

### Por que usar duas placas separadas (`HY-M154` + Módulo `PC817` 2 Canais)?
- Para a lógica Tri-State funcionar, o **canal do botão `+`** precisa ter **`CH1− = GND`** e o **canal do botão `−`** precisa ter **`IN1− = G2`**.
- Tanto a placa verde **`HY-M154` (4 canais)** quanto muitos lotes do módulo isolador **`PC817` de 2 canais (`3,6V–30V`)** unem na trilha de cobre os negativos de entrada dos canais (o teste em continuidade apita entre o `Parafuso 2` e o `Parafuso 4`). Se os dois botões fossem ligados na mesma placa com negativos unidos, **`G2` entraria em curto direto com `GND`**.
- **Solução adotada em produção (sem cortar trilha nem levantar pinos):** usar **1 canal de cada placa fisicamente separada** — Canal 1 da **`HY-M154`** para o **`+`** e Canal 1 do **módulo isolador `PC817` de 2 canais (`3,6V–30V`)** para o **`−`**.
  *(Nota: Caso o lote da placa possua negativos de entrada eletricamente isolados — sem apitar entre Parafuso 2 e Parafuso 4 — ou se você optar por seccionar a trilha traseira entre eles, ambos os canais podem operar em uma única placa.)*

### Princípio Matemático
Em cada canal de entrada das placas `PC817`, há um resistor limitador (`302` / $3\,\text{k}\Omega$), um **LED indicador SMD** ($V_F \approx 1.8\,\text{V}$) e o **LED infravermelho interno do PC817** ($V_F \approx 1.2\,\text{V}$) em série:
$$V_{\text{limiar por canal}} \approx 1.8\,\text{V} + 1.2\,\text{V} = \mathbf{3.0\,\text{V}}$$

Conectando a **Placa 1 (`+`)** entre `G2` e `GND`, e a **Placa 2 (`−`)** entre `+5V` e `G2`:

1. **Estado Repouso (`G2` em Alta Impedância / `INPUT` flutuante):**
   Os dois canais ficam em série entre `+5V` e `GND`. Para conduzir corrente pelos dois em série seriam necessários $\approx 3.0\,\text{V} + 3.0\,\text{V} = \mathbf{6.0\,\text{V}}$. Como a alimentação é de apenas **$5.0\,\text{V}$**, a corrente é **$0\,\text{mA}$** e **ambos os botões ficam soltos**.
2. **Acionar Botão `+` (`G2 = OUTPUT HIGH`, $3.3\,\text{V}$):**
   - **Placa 1 (`+`):** Recebe $3.3\,\text{V} - 0\,\text{V} = 3.3\,\text{V} \ge 3.0\,\text{V}$ $\rightarrow$ **LIGA (fecha botão `+`)**.
   - **Placa 2 (`−`):** Recebe $5.0\,\text{V} - 3.3\,\text{V} = 1.7\,\text{V} < 3.0\,\text{V}$ $\rightarrow$ **DESLIGADO**.
3. **Acionar Botão `−` (`G2 = OUTPUT LOW`, $0\,\text{V}$):**
   - **Placa 1 (`+`):** Recebe $0\,\text{V} - 0\,\text{V} = 0\,\text{V}$ $\rightarrow$ **DESLIGADO**.
   - **Placa 2 (`−`):** Recebe $5.0\,\text{V} - 0\,\text{V} = 5.0\,\text{V} \ge 3.0\,\text{V}$ $\rightarrow$ **LIGA (fecha botão `−`)**.

> [!IMPORTANT]
> **Teste rápido de bancada (com 5V e `G2` desconectado):**
> Com $+5\,\text{V}$ ligado na entrada `+` da Placa 2, `GND` ligado na entrada `−` da Placa 1, e a emenda em "Y" de `G2` entre elas deixada **solta no ar**, ambos os LEDs indicadores devem ficar **100% apagados**. Se acenderem fracamente com `G2` flutuando (caso raro de lote com LED indicador em paralelo com o PC817), adicione **1 diodo comum (`1N4148` ou `1N4007`) em série** na entrada de cada canal para elevar o limiar acima de $2.6\,\text{V}$ por canal.

---

## 4. Ligação Passo a Passo nas Duas Placas (`HY-M154` + `PC817 2 Canais`)

Em ambas as placas baseadas no `PC817`, os fotoacopladores permanecem **100% soldados normalmente (não levante nenhuma perna)**. Na entrada de cada canal, o resistor limitador **`302`** ($3\,\text{k}\Omega$) e o LED SMD já protegem o LED interno do `PC817` — portanto, **entre sempre pelos parafusos dos bornes verdes** (nunca direto nos pinos 1 e 2 do chip).

```text
  TRILHA INTERNA DE ENTRADA (CADA CANAL):
  Parafuso (+) ──> Pino 1 (PC817)
                   Pino 2 (PC817) ──> [Resistor 302] ──> [LED SMD] ──> Parafuso (−)
```

### Etapa A — Lado de Entrada (Bornes da Esquerda)

| Placa / Canal | Parafuso de Entrada | Onde ligar |
| :--- | :--- | :--- |
| **Placa 1: `HY-M154`**<br>(Canal 1 = Botão **`+`**) | **Parafuso 1 (`CH1+`)**<br>**Parafuso 2 (`CH1−`)** | **Fio Amarelo `G2`** do Grove (emenda em "Y" com Placa 2)<br>**Fio Preto `GND`** comum |
| **Placa 2: Módulo `PC817` 2 Canais (`3,6V–30V`)**<br>(Canal 1 = Botão **`−`**) | **Parafuso 1 (`IN1+`)**<br>**Parafuso 2 (`IN1−`)** | **Fio Vermelho `+5V`** do chicote<br>**Fio Amarelo `G2`** do Grove (mesma emenda em "Y" da Placa 1) |

*(Os demais canais não utilizados de cada placa ficam desconectados.)*

### Etapa B — Lado de Saída (Injeção de `GND` / Bypass `302` / 3 Fios até o Painel)

Os botões `+` e `−` do painel frontal Rinnai operam em lógica **Active-LOW (Pull-Up)**: um terminal de cada botão permanece em **nível Alto (`+5V`)** em repouso e o outro fecha com uma **trilha compartilhada de `GND`**.

Por isso, **ambos os canais operam como chaves coletor-aberto ligadas ao `GND`**, injetando **`GND` (`0V`)** no respectivo botão quando acionados:

1. **Configuração dos Jumpers Vermelhos de Saída:**
   - **Placa 1 (`HY-M154`, Canal 1 — `J1`):** Pode ser **mantido** (pois o `Parafuso 2` da entrada já é `GND`, referenciando o emissor ao `GND` comum) ou **removido** caso conecte o emissor por fio diretamente ao `GND`.
   - **Placa 2 (`PC817` 2 Canais, Canal 1 — `J1`):** **DEVE OBRIGATORIAMENTE SER REMOVIDO!** Na Placa 2, o negativo de entrada (`Parafuso 2 / IN1−`) é o sinal **`G2`** e **não** `GND`; manter esse jumper colocaria `G2` em curto com o `GND`.
2. **Bypass obrigatório nos resistores SMD `302` ($3\,\text{k}\Omega$) da saída (nas duas placas):**
   - Logo abaixo de cada conector do jumper vermelho há um resistor SMD **`302`** ($3\,\text{k}\Omega$) em série com a saída do fototransistor.
   - Para injetar um **`GND` franco ($\approx 0\,\text{V}$)** capaz de vencer o resistor de pull-up interno do painel, $3\,\text{k}\Omega$ em série impede a queda suficiente de tensão para registrar o clique.
   - **Solução:** Faça uma pequena ponte de solda (um pingo) por cima do resistor SMD **`302`** da saída no **Canal 1 da Placa 1 (`HY-M154`)** e no **Canal 1 da Placa 2 (`PC817 2 Canais`)**.
3. **Ligação com 3 fios até o painel frontal (`BTN+`, `BTN−` e `GND` compartilhado):**
   - **Emissores unidos ao `GND` comum:** Ligue o **`Borne OUT1−` (Pino 3 Emissor)** da Placa 2 ao **`Borne CH1−` (Pino 3 Emissor)** da Placa 1, seguindo com **1 único fio de `GND` compartilhado** até a trilha comum dos botões do painel (que compartilha o mesmo `GND` do Fio 2 do chicote).
   - **Botão `+` (`BTN+`):** Ligue o **`Borne CH1+` (Pino 4 Coletor)** da Placa 1 ao terminal em **Alta (`+5V` Pull-Up)** do **Botão `+`** no painel.
   - **Botão `−` (`BTN−`):** Ligue o **`Borne OUT1+` (Pino 4 Coletor)** da Placa 2 ao terminal em **Alta (`+5V` Pull-Up)** do **Botão `−`** no painel.

---

## 5. Diagrama Esquemático Completo (Vista Física de Cima)

```text
  CHICOTE RINNAI                     M5NanoC6 (Grove HY2.0-4P)
  ──────────────                     ─────────────────────────
  Fio 1 (+5V) ──────────────────┬──> [ 5V  ] (Vermelho)
                                │
  Fio 2 (GND) ──────────────┬───┼──> [ GND ] (Preto)
                            │   │
  Fio 3 (TX) ──[ 10 kΩ ]──┬─┼───┼──> [ G1  ] (Branco - RMT RX 1 µs)
                          │ │   │
                       [20 kΩ]  │    Fio 4 (RX Central) ── (Não ligado ao M5NanoC6)
                          │ │   │
                         GND│   │
                            │   │    PLACA 1: HY-M154 (Canal 1 = Botão "+")
                            │   │    ┌────────────────────────────────────────────────────┐
             ┌──────────────┼───┼───>│ Paraf. 1 (CH1+) ──> PC817 #1 ──[302]──[LED]──┐     │
             │              └───┼───>│ Paraf. 2 (CH1−) <────────────────────────────┘(GND)│
             │                  │    ├────────────────────────────────────────────────────┤
             │                  │    │ Saída (J1 opcional | SMD 302 com BYPASS 0 Ω):      │
             │                  │    │  • Borne CH1+  (Pino 4 Coletor) ───────────────────┤──> BTN+ (+5V Pull-Up)
             │                  │    │  • Borne CH1−  (Pino 3 Emissor) ───────────────────┤──┐
             │                  │    └────────────────────────────────────────────────────┘  │
  [ G2 ] ────┤ (Emenda "Y")     │                                                            │ (ponte GND
  (Amarelo)  │                  │    PLACA 2: Módulo PC817 2 Canais (Canal 1 = Botão "−")    │  comum)
             │                  │    ┌────────────────────────────────────────────────────┐  │
             │                  └───>│ Paraf. 1 (IN1+) ──> PC817 #2 ──[302]──[LED]──┐(+5V)│  │
             └──────────────────────>│ Paraf. 2 (IN1−) <────────────────────────────┘     │  │
                                     ├────────────────────────────────────────────────────┤  │
                                     │ Saída (J1 REMOVIDO | SMD 302 com BYPASS 0 Ω):      │  │
                                     │  • Borne OUT1+ (Pino 4 Coletor) ───────────────────┤──┼──> BTN− (+5V Pull-Up)
                                     │  • Borne OUT1− (Pino 3 Emissor) ───────────────────┤──┴──> GND Painel
                                     └────────────────────────────────────────────────────┘
                                       ▲ Total até o painel frontal: 3 fios (BTN+, BTN−, GND)
```

> [!TIP]
> **Como confirmar no painel qual terminal de cada botão é o lado em Alta (`+5V`) e qual é o `GND` compartilhado:**
> Como o fototransistor NPN do PC817 conduz em apenas um sentido (do Coletor Pino 4 para o Emissor Pino 3 aterrado):
> 1. Com o multímetro em **continuidade ($\Omega$)** (painel desligado), confirme que um dos lados do botão `+` e um dos lados do botão `−` estão unidos na mesma trilha compartilhada de **`GND`** (continuidade com o **Fio 2 Preto** do chicote). Esse lado comum é onde ficam os **Emissores (Pino 3 / `Borne −`)**.
> 2. Com o aquecedor energizado e o multímetro em **Volts DC ($20\,\text{V}$)**, deixe a **ponta Preta (`-`)** fixa no `GND` (Fio 2) e encoste a **ponta Vermelha (`+`)** nos terminais do botão `+`:
>    - O terminal que marcar **tensão positiva constante (`+5V` ou `+3.3V`)** em repouso e cair para **`0V`** ao apertar o botão físico é o lado em **Alta (Pull-Up)** $\rightarrow$ ligue no **Pino 4 (Coletor / `Borne CH1+`)** da Placa 1.
> 3. Repita o passo 2 para o botão `−` e ligue o lado em **Alta** no **Pino 4 (Coletor / `Borne OUT1+`)** da Placa 2.

---

## 6. Integração com Zigbee2MQTT, OTA e Homebridge (HomeKit)

### Passo 1: Gravar o Firmware no M5NanoC6
Use o projeto PlatformIO em [firmware_nanoc6/platformio.ini](../firmware_nanoc6/platformio.ini) (ou o sketch Arduino em [rinnai_nanoc6_zigbee.ino](../firmware_nanoc6/rinnai_nanoc6_zigbee/rinnai_nanoc6_zigbee.ino)):
- Na Arduino IDE: selecione **M5NanoC6**, **Zigbee Mode: `Zigbee ED (End Device)`**, **Partition Scheme: `Zigbee 4MB with SPIFFS`**, **USB CDC On Boot: `Enabled`**.
- Para parear pela primeira vez (ou resetar a rede), segure o botão integrado **`G9`** por **5 segundos** até o LED RGB piscar em **azul**.
- Atualizações futuras podem ser enviadas remotamente por Zigbee OTA conforme [docs/ota.md](ota.md).

### Passo 2: Instalar o External Converter no Zigbee2MQTT (2.x ESM)
1. Copie o arquivo [z2m/rinnai_nanoc6.mjs](../z2m/rinnai_nanoc6.mjs) para a pasta `data/external_converters/` do seu Zigbee2MQTT.
2. Reinicie o Zigbee2MQTT e ative **Permit Join**.
3. O dispositivo aparecerá como **`Rinnai-NanoC6`** (`RinnaiCustom`), expondo um **Termostato (`climate`)** com:
   - `local_temperature`: Temperatura real lida na linha `TX` da central (decodificação RMT + CRC-16)
   - `occupied_heating_setpoint`: Controle deslizante limitado à faixa segura de **35°C a 43°C** (passo `1°C`)
   - `system_mode`: `heat` quando a chama está acesa; `off` em qualquer outro caso
   - `running_state`: `heat` ("Aquecendo" em laranja no HomeKit) quando a chama estiver acesa, e `idle` quando apagada.

### Passo 3: Homebridge (`homebridge-z2m`)
O plugin `homebridge-z2m` detecta automaticamente a exposição `climate` com `local_temperature`, `occupied_heating_setpoint`, `system_mode` e `running_state` e cria o acessório **Termostato** nativo no app **Casa (HomeKit)**:
- O slider do HomeKit respeitará a faixa segura de **35°C a 43°C** (passo de `1°C`).
- Durante o banho (chama acesa), o ícone do HomeKit muda para **Aquecendo** (laranja).
- Se você tentar aumentar a temperatura acima da trava de segurança da Rinnai durante o banho (ex: $>42^\circ\text{C}$ com água correndo), após **2 tentativas sem mudança no painel**, a trava anti-loop interrompe os cliques e sincroniza o slider do HomeKit de volta para a temperatura real.
