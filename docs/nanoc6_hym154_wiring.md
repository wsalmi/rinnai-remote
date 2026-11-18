# Guia de Montagem Elétrica: M5NanoC6 + Módulo HY-M154 (PC817) + Zigbee2MQTT / HomeKit

Este guia documenta a arquitetura definitiva para controlar o aquecedor a gás **Rinnai** utilizando um **M5Stack M5NanoC6** (ESP32-C6 com Zigbee nativo), integrando-o ao **Zigbee2MQTT** e **Homebridge** (Apple HomeKit) usando apenas a porta **Grove** de 4 vias e o módulo optoacoplador **HY-M154 (4 canais PC817)**.

---

## 1. Por que abandonamos a injeção digital no pino RX?

Nos testes anteriores, ao conectar a saída do microcontrolador no fio `RX` da central (onde o painel frontal original envia os quadros de botões), ocorria **colisão elétrica no barramento (*bus contention*)**:

- O painel original da Rinnai mantém a linha `RX` ativamente controlada para transmitir quadros contínuos de status/handshake.
- Quando o ESP32 colocava seu pino em modo saída (*Push-Pull*) e tentava injetar pulsos, os dois transmissores entravam em conflito de nível lógico na mesma linha, corrompendo o quadro de sincronismo e fazendo o painel frontal entrar em estado de erro/pânico.

### A Solução: Leitura Passiva + Atuação Isolada nos Botões (`+` e `-`)
1. **Telemetria 100% Passiva (`G1`):** O M5NanoC6 apenas **escuta** o fio `TX` da central através de um divisor de tensão resistivo (alta impedância, sem jamais transmitir nada no barramento).
2. **Atuação via Contato Seco Optoisolado (`G2` + HY-M154):** Em vez de gerar pacotes no fio `RX`, fechamos eletronicamente os contatos dos botões físicos **`+`** e **`-`** do painel frontal usando dois canais do módulo **HY-M154** (PC817). Assim, o microcontrolador original do próprio painel gera os quadros válidos, mantém 100% das travas de segurança originais da Rinnai e preserva o bip sonoro nativo.

---

## 2. Mapeamento do M5NanoC6 e Conector Grove

O M5NanoC6 possui um conector Grove (HY2.0-4P) com apenas **2 GPIOs** expostos (`G1` e `G2`), além de `+5V` e `GND`:

| Pino Grove (Cor Padrão) | Pino M5NanoC6 | Função no Projeto |
| :--- | :--- | :--- |
| **Preto** | `GND` | Terra comum com a central Rinnai (`Fio 2 - Preto`) |
| **Vermelho** | `5V` | Alimentação $+5\,\text{V}$ vinda do chicote da central (`Fio 1 - Vermelho`) |
| **Branco** | **`G1` (`GPIO 1`)** | **Leitura Passiva (`RX` UART 2800 baud)** vinda do `TX` da central via divisor $10\,\text{k}\Omega / 20\,\text{k}\Omega$ |
| **Amarelo** | **`G2` (`GPIO 2`)** | **Saída Tri-State** que aciona os 2 canais do HY-M154 (`+` e `-`) em um único pino |

### Periféricos Integrados Utilizados no M5NanoC6
- **`G9` (Botão físico):** Segurar por **5 segundos** executa Factory Reset do Zigbee e entra em modo de pareamento.
- **`G19` + `G20` (LED RGB WS2812):** `G19` habilita a alimentação do LED; `G20` mostra o status:
  - **Azul piscando:** Aguardando pareamento Zigbee
  - **Verde suave:** Conectado e em repouso/standby
  - **Laranja:** Chama ativa (aquecendo água no banho)
  - **Piscada rápida Ciano / Magenta:** Simulando clique no botão `+` ou `-`
- **`G7` (LED Azul discreto):** Pisca rapidamente a cada quadro válido recebido da central.

---

## 3. O Circuito Tri-State no Pino `G2` com o Módulo HY-M154

Como temos apenas **1 pino livre (`G2`)** para acionar **2 botões (`+` e `-`)**, utilizamos a técnica de **Lógica Tri-State (3 estados)** aproveitando a queda de tensão dos LEDs internos do módulo HY-M154:

### Princípio Matemático
Em cada canal de entrada do módulo **HY-M154**, há um resistor limitador, um **LED indicador SMD** ($V_F \approx 1.8\,\text{V}$) e o **LED infravermelho interno do PC817** ($V_F \approx 1.2\,\text{V}$) em série:
$$V_{\text{limiar por canal}} \approx 1.8\,\text{V} + 1.2\,\text{V} = \mathbf{3.0\,\text{V}}$$

Quando conectamos o **Canal 1 (`+`)** entre `G2` e `GND`, e o **Canal 2 (`-`)** entre `+5V` e `G2`:

1. **Estado Repouso (`G2` em Alta Impedância / `INPUT` flutuante):**
   Os dois canais ficam em série entre `+5V` e `GND`. Para conduzir corrente pelos dois em série seriam necessários $\approx 3.0\,\text{V} + 3.0\,\text{V} = \mathbf{6.0\,\text{V}}$. Como a alimentação é de apenas **$5.0\,\text{V}$**, a corrente é **$0\,\text{mA}$** e **ambos os botões ficam soltos**.
2. **Acionar Botão `+` (`G2 = OUTPUT HIGH`, $3.3\,\text{V}$):**
   - **Canal 1 (`+`):** Recebe $3.3\,\text{V} - 0\,\text{V} = 3.3\,\text{V} \ge 3.0\,\text{V}$ $\rightarrow$ **LIGA (fecha botão `+`)**.
   - **Canal 2 (`-`):** Recebe $5.0\,\text{V} - 3.3\,\text{V} = 1.7\,\text{V} < 3.0\,\text{V}$ $\rightarrow$ **DESLIGADO**.
3. **Acionar Botão `-` (`G2 = OUTPUT LOW`, $0\,\text{V}$):**
   - **Canal 1 (`+`):** Recebe $0\,\text{V} - 0\,\text{V} = 0\,\text{V}$ $\rightarrow$ **DESLIGADO**.
   - **Canal 2 (`-`):** Recebe $5.0\,\text{V} - 0\,\text{V} = 5.0\,\text{V} \ge 3.0\,\text{V}$ $\rightarrow$ **LIGA (fecha botão `-`)**.

> [!IMPORTANT]
> **Verificação obrigatória do lote do seu HY-M154 antes de ligar no aquecedor:**
> A maioria das placas HY-M154 coloca o LED indicador SMD em **série** com o PC817 (limiar $\sim 3.0\,\text{V}$). Entretanto, existem clones chineses onde o LED indicador está em **paralelo** com o PC817 (nesse caso, o limiar cai para apenas $\sim 1.2\,\text{V}$ e os dois LEDs acenderiam juntos quando `G2` estivesse solto!).
>
> **Teste rápido de bancada (com 5V na entrada e `G2` desconectado):**
> Ligue $+5\,\text{V}$ no Parafuso 3 (`CH2+`), `GND` no Parafuso 2 (`CH1-`), faça o jumper ligando o Parafuso 1 (`CH1+`) ao Parafuso 4 (`CH2-`), e deixe o fio `G2` **solto no ar**. Os dois LEDs (`CH1` e `CH2`) devem ficar **100% apagados**. Se acenderem fraquinho com `G2` desconectado, basta adicionar **2 diodos comuns (1N4148 ou 1N4007) em série** na entrada de cada canal para elevar o limiar acima de $2.6\,\text{V}$ por canal.

---

## 4. Passo a Passo na Placa HY-M154 Verde (4 Blocos / 8 Parafusos)

A placa **HY-M154 ("817 Module")** possui 4 canais independentes com **4 blocos verdes de 2 parafusos** na entrada (esquerda) e 4 na saída (direita). Os chips **PC817 #1** e **PC817 #2** ficam **100% soldados normalmente (não levante nenhuma perna!)**, pois na trilha real dessa placa o resistor limitador **`302`** ($3\,\text{k}\Omega$) e o LED SMD ficam **entre a Perna 2 e o parafuso de baixo**:

```text
  TRILHA INTERNA DA PLACA HY-M154 (CADA CANAL):
  Parafuso de Cima (+) ──> Pino 1 (PC817)
                           Pino 2 (PC817) ──> [Resistor 302] ──> [LED SMD] ──> Parafuso de Baixo (−)
```

> [!CAUTION]
> **Nunca ligue fios direto na Perna 2 do PC817 nesta placa verde:** isso pularia o resistor `302` de $3\,\text{k}\Omega$ e queimaria o fotoacoplador com 5V. Entre sempre pelos parafusos dos bornes verdes!

### Etapa A — Lado de Entrada (4 Primeiros Parafusos da Esquerda)
1. **Remova obrigatoriamente o jumper vermelho do Canal 2 (`J2`)**, pois o `Parafuso 4` receberá o sinal `G2` (não `GND`). No **Canal 1 (`J1`)**, como o `Parafuso 2` já é `GND`, o jumper pode ser mantido para referenciar o emissor da saída ao `GND` comum (ou removido caso prefira ligar os emissores diretamente à trilha de `GND` dos botões).
2. **Teste de multímetro (5 segundos):** em continuidade ($\Omega$), verifique que o **Parafuso 2** (`CH1 −`) e o **Parafuso 4** (`CH2 −`) **não** estão em curto entre si na placa.
3. Faça a ligação direta nos 4 primeiros parafusos da esquerda (de cima para baixo):

| Parafuso (Esquerda, de cima para baixo) | Função no HY-M154 | Onde ligar |
| :--- | :--- | :--- |
| **Parafuso 1** (Bloco 1, Cima) | `CH1 +` (Botão `+`) | **Fio Amarelo `G2`** do Grove (emendado em "Y" com o Parafuso 4) |
| **Parafuso 2** (Bloco 1, Baixo) | `CH1 −` (Botão `+`) | **Fio Preto `GND`** comum |
| **Parafuso 3** (Bloco 2, Cima) | `CH2 +` (Botão `−`) | **Fio Vermelho `+5V`** |
| **Parafuso 4** (Bloco 2, Baixo) | `CH2 −` (Botão `−`) | **Fio Amarelo `G2`** do Grove (mesmo fio do Parafuso 1) |

### Etapa B — Lado de Saída (Injeção de `GND` / Active-LOW com Pull-Up)
Os botões `+` e `−` do painel Rinnai operam em lógica **Active-LOW (Pull-Up)**: um terminal de cada botão permanece em **nível Alto (`+5V`)** em repouso e o outro fecha com uma **trilha compartilhada de `GND`**.

Por isso, **ambos os canais do HY-M154 funcionam como chaves coletor-aberto ligadas ao `GND`**, injetando **`GND` (`0V`)** no respectivo botão quando acionados:
- Mesmo que na **entrada** o pino `G2` use `HIGH` (`3.3V`) para disparar o Canal 1 (`+`) e `LOW` (`0V`) para disparar o Canal 2 (`−`), a **saída** dos PC817 é galvanicamente separada da entrada e **ambas injetam `GND` (`0V`)** ao fechar o Coletor (Pino 4) para o Emissor (Pino 3).

1. **Jumpers vermelhos e unificação dos Emissores (`GND` comum):**
   - **Canal 1 (`J1`):** Como o `Parafuso 2` da entrada já está no `GND`, o jumper vermelho **`J1`** pode ser mantido (ligando automaticamente o **Pino 3 Emissor / `Borne CH1−`** ao `GND` comum).
   - **Canal 2 (`J2`):** **DEVE obrigatoriamente permanecer REMOVIDO**, pois o `Parafuso 4` da entrada é o sinal `G2` e **não** `GND` (manter `J2` colocaria `G2` em curto com `GND`).
   - Faça uma **ponte com um fio** ligando o **`Borne CH2−` (Pino 3 Emissor)** ao **`Borne CH1−` (Pino 3 Emissor / `GND`)** — ou ligue ambos os emissores diretamente à trilha compartilhada de `GND` dos botões do painel (que compartilha o mesmo `GND` do Fio 2 do chicote).
2. **Ligação simplificada nos botões (1 fio por botão + `GND` compartilhado):**
   - **`Borne CH1+` (Pino 4 Coletor):** Ligue diretamente ao lado em **Alta (`+5V`)** do **Botão `+`** no painel.
   - **`Borne CH2+` (Pino 4 Coletor):** Ligue diretamente ao lado em **Alta (`+5V`)** do **Botão `−`** no painel.
3. **Bypass obrigatório nos resistores SMD `302` ($3\,\text{k}\Omega$) da saída (Canais 1 e 2):**
   - Logo abaixo de cada conector onde ficava o jumper vermelho há um resistor SMD `302` ($3\,\text{k}\Omega$) em série com o borne de saída.
   - Para injetar um **`GND` franco ($\approx 0\,\text{V}$)** capaz de vencer o pull-up interno do painel, $3\,\text{k}\Omega$ em série impede que a tensão caia o suficiente para o microcontrolador da Rinnai reconhecer o clique.
   - **Solução:** Faça uma pequena ponte de solda (um pingo) por cima do resistor SMD `302` da saída nos Canais 1 e 2, **ou** solde os fios diretamente nas pernas **Pino 3 (Emissor $\rightarrow$ `GND`)** e **Pino 4 (Coletor $\rightarrow$ lado Alta do botão)** de cada PC817.

---

## 5. Diagrama Esquemático Completo (Vista Física de Cima)

```text
  CHICOTE RINNAI                     M5NanoC6 (Grove)
  ──────────────                     ────────────────
  Fio 1 (+5V) ──────────────────┬──> [ 5V  ] (Vermelho)
                                │
  Fio 2 (GND) ──────────────┬───┼──> [ GND ] (Preto)
                            │   │
  Fio 3 (TX) ──[ 10 kΩ ]──┬─┼───┼──> [ G1  ] (Branco - RX 2800 baud)
                          │ │   │
                       [20 kΩ]  │
                          │ │   │    PLACA VERDE HY-M154 (Lado Entrada - 4 Primeiros Parafusos)
                         GND│   │    ──────────────────────────────────────────────────────────
                            │   │    ┌────────────────────────────────────────────────────────┐
             ┌──────────────┼───┼───>│ Paraf. 1 (CH1+) ──> PC817 #1 ──[302]──[LED1]──┐        │
             │              └───┼───>│ Paraf. 2 (CH1−) <─────────────────────────────┘ (GND)  │
             │                  │    │                                                        │
             │                  └───>│ Paraf. 3 (CH2+) ──> PC817 #2 ──[302]──[LED2]──┐ (+5V)  │
             │  ┌───────────────────>│ Paraf. 4 (CH2−) <─────────────────────────────┘        │
             │  │                    └────────────────────────────────────────────────────────┘
  [ G2 ] ────┴──┴── (Emenda em "Y": liga G2 no Parafuso 1 E no Parafuso 4)

  PLACA VERDE HY-M154 (Lado Saída — Injeção de GND / Coletor Aberto)
  ──────────────────────────────────────────────────────────────────
  • Jumper J1 (CH1): pode ser MANTIDO (Parafuso 2 já é GND)
  • Jumper J2 (CH2): REMOVIDO OBRIGATORIAMENTE (Parafuso 4 é G2, nunca ligue ao GND!)
  • Resistores SMD 302 de saída (CH1 e CH2): com BYPASS (pingo de solda) para GND franco (0 Ω)

  PC817 #1 (Botão +):
    Borne CH1 + (Pino 4 Coletor) ──> Terminal em ALTA (+5V) do Botão "+" no painel
    Borne CH1 − (Pino 3 Emissor) ──┬──> GND comum (via J1 ou trilha compartilhada GND do painel)
                                   │ (ponte entre Emissores)
  PC817 #2 (Botão −):              │
    Borne CH2 − (Pino 3 Emissor) ──┘
    Borne CH2 + (Pino 4 Coletor) ──> Terminal em ALTA (+5V) do Botão "−" no painel
```

> [!TIP]
> **Como confirmar no painel qual terminal de cada botão é o lado em Alta (`+5V`) e qual é o `GND` compartilhado:**
> Como o fototransistor NPN do PC817 conduz em apenas um sentido (do Coletor Pino 4 para o Emissor Pino 3 aterrado):
> 1. Com o multímetro em **continuidade ($\Omega$)** (painel desligado), confirme que um dos lados do botão `+` e um dos lados do botão `−` estão unidos na mesma trilha compartilhada de **`GND`** (continuidade com o **Fio 2 Preto** do chicote). Esse lado comum é onde ficam os **Emissores (Pino 3 / `Borne −`)**.
> 2. Com o aquecedor energizado e o multímetro em **Volts DC ($20\,\text{V}$)**, deixe a **ponta Preta (`-`)** fixa no `GND` (Fio 2) e encoste a **ponta Vermelha (`+`)** nos terminais do botão `+`:
>    - O terminal que marcar **tensão positiva constante (`+5V` ou `+3.3V`)** em repouso e cair para **`0V`** ao apertar o botão físico é o lado em **Alta (Pull-Up)** $\rightarrow$ ligue no **Pino 4 (Coletor / `Borne CH1+`)**.
> 3. Repita o passo 2 para o botão `−` e ligue o lado em **Alta** no **Pino 4 (Coletor / `Borne CH2+`)**.

---

## 6. Integração com Zigbee2MQTT e Homebridge (HomeKit)

### Passo 1: Gravar o Firmware no M5NanoC6
Use o projeto PlatformIO em [firmware_nanoc6/](file:///Users/wesleysalmi/source-me/rinnai-remote/firmware_nanoc6/platformio.ini) (ou o sketch Arduino em [rinnai_nanoc6_zigbee.ino](file:///Users/wesleysalmi/source-me/rinnai-remote/firmware_nanoc6/rinnai_nanoc6_zigbee/rinnai_nanoc6_zigbee.ino)):
- Na Arduino IDE: selecione **M5NanoC6**, **Zigbee Mode: `Zigbee ED (End Device)`**, **Partition Scheme: `Zigbee 4MB with SPIFFS`**, **USB CDC On Boot: `Enabled`**.
- Para parear pela primeira vez (ou resetar a rede), segure o botão integrado **`G9`** por **5 segundos** até o LED RGB piscar em **azul**.

### Passo 2: Instalar o External Converter no Zigbee2MQTT
1. Copie o arquivo [rinnai_nanoc6.js](file:///Users/wesleysalmi/source-me/rinnai-remote/z2m/rinnai_nanoc6.js) para a pasta `data/external_converters/` do seu Zigbee2MQTT (ou referencie-o em `external_converters:` no `configuration.yaml` caso use versão antiga do Z2M).
2. Reinicie o Zigbee2MQTT e ative **Permit Join**.
3. O dispositivo aparecerá como **`Rinnai-NanoC6`** (`RinnaiCustom`), expondo um **Termostato (`climate`)** com:
   - `local_temperature`: Temperatura real lida na linha `TX` da central
   - `occupied_heating_setpoint`: Controle deslizante limitado de **35°C a 48°C** (passo `1°C`)
   - `system_mode`: Espelha o estado físico do botão Power do painel (`heat` / `off`)
   - `running_state`: `heat` ("Aquecendo" em laranja no HomeKit) quando a chama estiver acesa, e `idle` quando apagada.

### Passo 3: Homebridge (`homebridge-z2m`)
O plugin `homebridge-z2m` detecta automaticamente a exposição `climate` com `local_temperature`, `occupied_heating_setpoint`, `system_mode` e `running_state` e cria o acessório **Termostato** nativo no app **Casa (HomeKit)**:
- O slider do HomeKit respeitará a faixa segura de **35°C a 48°C** (passo de 1°C).
- Durante o banho (chama acesa), o ícone do HomeKit muda para **Aquecendo** (laranja).
- Se você tentar aumentar a temperatura acima da trava de segurança da Rinnai durante o banho (ex: $>42^\circ\text{C}$ com água correndo), após **2 tentativas sem mudança no painel**, a trava anti-loop interrompe os cliques e sincroniza o slider do HomeKit de volta para a temperatura real.
