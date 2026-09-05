# Especificação do Protocolo Digital Rinnai

## 1. Estrutura dos Pacotes

O barramento transmite quadros estruturados em pacotes de **6 bytes** (48 bits), transmitidos a cada **150 ms a 200 ms**.

### Regras de Codificação
* **Ordem de Transmissão:** LSB-First (bit menos significativo primeiro).
* **Paridade:** O 8º bit (MSB, bit 7) de cada byte é um **bit de paridade ímpar (*odd parity*)**.
  $$\text{Paridade}(B) = \left( \sum_{i=0}^{6} B_i \right) \pmod 2 \ne 1$$
* **Checksum (LRC):** O 6º byte ($\text{Byte}[5]$) é a operação XOR entre os 5 primeiros bytes:
  $$\text{Byte}[5] = \text{Byte}[0] \oplus \text{Byte}[1] \oplus \text{Byte}[2] \oplus \text{Byte}[3] \oplus \text{Byte}[4]$$

---

## 2. Pacote de Telemetria (Central $\rightarrow$ Display)

| Byte | Nome | Descrição e Mapeamento de Bits |
| :---: | :--- | :--- |
| **0** | **Cabeçalho / ID** | `[7:4]` Prioridade / ID do remetente.<br>`[3:0]` Fixo em `0x7` (indica origem: Central Principal). |
| **1** | **Estado Operacional** | `Bit 6 (0x40)`: **Sistema Ligado/Standby** ($1 = \text{Ligado}$, $0 = \text{Desligado}$).<br>`Bit 3 (0x08)`: Escala de temperatura especial (utilizado para 35°C). |
| **2** | **Chama & Temperatura** | `Bit 4 (0x10)`: **Chama Ativa / Queimador em Uso** ($1 = \text{Fogo Aceso}$, $0 = \text{Apagado}$).<br>`[3:0]` **Código de Temperatura** ajustada (ver tabela abaixo). |
| **3** | **Potência & Exaustor** | Modulação da válvula de gás proporcional (Low/Med/High) e estado/feedback do ventilador de exaustão. |
| **4** | **Diagnóstico / Erros** | `0x20` em repouso normal sem falhas.<br>Outros códigos representam erros (Ex: `11` = falha de ignição, `12` = perda de chama, `90` = exaustão obstruída). |
| **5** | **Checksum** | XOR dos Bytes 0 a 4. |

### Tabela de Mapeamento de Temperatura

A temperatura ajustada na central segue a equação base:
$$\text{Temperatura} = 37 + \text{Código}$$

| Código (`Byte 2 [3:0]`) | Temperatura Exibida | Observação |
| :---: | :---: | :--- |
| `0x0` + Flag `0x08` (Byte 1) | **35°C** | Modo temperatura mínima de verão |
| `0x0` | **37°C** | Base linear |
| `0x1` | **38°C** | Linear |
| `0x2` | **39°C** | Linear |
| `0x3` | **40°C** | Linear |
| `0x4` | **41°C** | Linear |
| `0x5` | **42°C** | **Ponto de teste comprovado em captura real** |
| `0x6` | **43°C** | Linear |
| `0x7` | **44°C** | Linear |
| `0x8` | **45°C** | Linear |
| `0x9` | **46°C** | Linear |
| `0xA` | **47°C** | Linear |
| `0xB` | **48°C** | Linear |
| `0xC` | **50°C** | Salto de escala |
| `0xD` | **55°C** | Salto de escala |
| `0xE` | **60°C** | Temperatura máxima padrão |

---

## 3. Pacote de Comandos (Painel $\rightarrow$ Central)

Transmitido na linha de retorno para enviar comandos de acionamento do usuário:

| Byte | Nome | Descrição |
| :---: | :--- | :--- |
| **0** | **ID do Painel** | `[7:4]` Fixo em `0x0` (origem: painel/remoto).<br>`[3:0]` ID de registro do painel. |
| **1** | **Comandos do Sistema** | `Bit 0 (0x01)`: Botão **Power (Liga/Desliga)** pressionado.<br>`Bit 2 (0x04)`: Botão **Prioridade** pressionado. |
| **2** | **Ajuste de Temperatura** | `Bit 0 (0x01)`: Botão **Temperatura +** pressionado.<br>`Bit 1 (0x02)`: Botão **Temperatura -** pressionado. |
| **3** | **Handshake** | Estado de sincronismo da comunicação. |
| **4** | **Constante** | Fixo em `0xBF`. |
| **5** | **Checksum** | XOR dos Bytes 0 a 4. |
