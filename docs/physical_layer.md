# Camada Física (Physical Layer) - Barramento Digital Rinnai

## 1. Conexão e Pinout do Chicote

O chicote que interliga a central eletrônica principal ao painel frontal digital (ou controle remoto) possui 4 vias:

| Fio | Sinal | Descrição | Nível de Tensão |
| :--- | :--- | :--- | :--- |
| **1** | `+5V` | Alimentação positiva DC fornecida pela central | +5.0 V |
| **2** | `GND` | Referência de terra comum | 0 V |
| **3** | `TX` (Central) / `RX` (Display) | Linha de telemetria enviada da central ao painel | 5V TTL |
| **4** | `RX` (Central) / `TX` (Display) | Linha de comandos enviada do painel à central | 5V TTL |

> [!CAUTION]
> **Compatibilidade de Nível Lógico (5V vs 3.3V):**
> Os sinais da central operam em **5V TTL**. O ESP32 opera com níveis lógicos CMOS de **3.3V** e seus GPIOs não são tolerantes a 5V contínuo.
> - **Para a linha de recepção no ESP32 (GPIO 5):** Recomenda-se o uso de um divisor de tensão resistivo ($10\,\text{k}\Omega / 20\,\text{k}\Omega$) ou um conversor de nível bidirecional tipo MOSFET.
> - **Para a linha de transmissão do ESP32 (GPIO 15):** O nível de saída em ALTO (3.3V) do ESP32 geralmente atinge o limiar mínimo de nível ALTO ($V_{IH} \approx 2.0\text{V}$) da central de 5V TTL, mas para maior robustez pode-se utilizar um buffer/transistor.

---

## 2. Características de Modulação e Temporização

O barramento **não utiliza** o padrão assíncrono UART (8N1). O protocolo utiliza **modulação por comprimento de pulso (Variable Pulse Length / PWM)**.

### A. Base de Tempo (Symbol Timing)
* **Tempo elementar de bit ($T$):** $\approx 368\,\mu\text{s} \text{ a } 370\,\mu\text{s}$.
* **Taxa equivalente:** $\approx 2700 \text{ bps}$ (ou $\approx 2800 \text{ bps}$ com tolerância de clock).
* **Período do símbolo:** $\approx 600\,\mu\text{s} \text{ a } 740\,\mu\text{s}$.

```
                 Padrão de Pulsos (Deltas de Hardware)
Bit 0:
      ┌──────┐
      │~370µs│             ~370µs
  ────┘      └─────────────────────────────────

Bit 1:
      ┌──────┐
      │~370µs│                      ~1110µs (3x)
  ────┘      └─────────────────────────────────

Reset / Início de Quadro:
      ┌────────────────────────────────────────┐
      │           > 3500µs a 7200µs            │ (Silêncio / Linha em Repouso)
  ────┘                                        └─────
```

### B. Medições Reais Capturadas em Hardware (`sniff 5`)
Ao monitorar as transições no pino RX (GPIO 5) através do Bus Pirate:
* `delta = ~368 µs` ($1\times$ tempo base)
* `delta = ~1108 µs` ($3\times$ tempo base)
* `delta = ~1847 µs` ($5\times$ tempo base)
* `delta = ~7200 µs` (intervalo de repouso entre pacotes)
