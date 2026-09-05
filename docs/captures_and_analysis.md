# Análise de Capturas Reais (Case Studies)

Este documento registra as capturas efetuadas em bancada utilizando o ESP32 / Bus Pirate (firmware Bruce) conectado aos pinos **RX (GPIO 5)** e **TX (GPIO 15)**.

---

## 1. Cenário 1: Central em Standby (Repouso)

### Captura em Modo Bridge UART a 2800 baud:
```text
QUUUUUEQUEDTTUEUEEQUUUEU\x0cEQUEDQUDEDQ5EUUUUTUUTQD\x0c
```

### Representação Hexadecimal:
```hex
51 55 55 55 55 55 45 51 55 45 44 54 54 55 45 55 45 45 51 55 55 55 45 55 0C 
45 51 55 45 44 51 55 44 45 44 51 35 45 55 55 55 55 54 55 55 54 51 44 0C
```

### Análise:
* O caractere predominante é `U` (`0x55 = 01010101_b`), demonstrando uma linha alternada em pulsos curtos e simétricos.
* A ausência de flags de alta duração no início do pacote indica que o queimador está desligado.
* `Byte 2 [4]` está em `0` (Chama desligada).

---

## 2. Cenário 2: Chuveiro Ligado com Temperatura em 42°C

### Captura em Modo Bridge UART a 2800 baud:
```text
Q\x15UUEUU\x15TU\x15DQ\x15E\x00\x00U\x14DQ\x15Q\x15Q\x15D\x15DTTEED\x00\x15Q\x14U\x11EDQUDED\x15Q5\x00E\x15UU\x11\x11\x15TE\x14TU\x14DUQ\x03\x00
```

### Representação Hexadecimal:
```hex
51 15 55 55 45 55 55 15 54 55 15 44 51 15 45 00 00 55 14 44 51 15 51 15 51 
15 44 15 44 54 54 45 45 44 00 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 
00 45 15 55 55 11 11 15 54 45 14 54 55 14 44 55 51 03 00
```

### Análise Comparativa do Byte `0x15`:
Imediatamente ao abrir o chuveiro, surge o byte **`0x15`**:
$$\mathbf{0x15} = \mathbf{0001 \ 0101}_b$$

1. **Nibble Alto (`0x1`):**
   * Corresponde a `Bit 4 = 1` $\rightarrow$ **Chama Ativa / Em Uso**.
2. **Nibble Baixo (`0x5`):**
   * Corresponde ao **Código 5** da tabela de temperatura:
   $$\text{Temperatura} = 37 + 5 = \mathbf{42^\circ\text{C}}$$
   * Confirmação direta com o valor ajustado fisicamente no visor do aquecedor.

---

## 3. Análise de Transições e Temporizações (`sniff 5`)

O comando `sniff 5` capturou as larguras de pulso dos símbolos:
```text
Pin 5: HIGH -> LOW  | delta=368us   <-- Bit base (1x tempo)
Pin 5: LOW  -> HIGH | delta=370us
Pin 5: HIGH -> LOW  | delta=368us
Pin 5: LOW  -> HIGH | delta=1108us  <-- Pulso alargado (3x tempo = 3 * 369.3us)
Pin 5: HIGH -> LOW  | delta=1104us
Pin 5: LOW  -> HIGH | delta=369us
Pin 5: LOW  -> HIGH | delta=1847us  <-- Pulso longo (5x tempo)
...
Pin 5: HIGH -> LOW  | delta=7199us  <-- Silêncio de fim de quadro (Inter-frame gap)
```

Essa medição confirma que a base de tempo do oscilador da central é de **$370\,\mu\text{s}$**, justificando a captura a **2700-2800 baud** na UART e determinando os tempos ideais para o firmware receptor no ESP32.
