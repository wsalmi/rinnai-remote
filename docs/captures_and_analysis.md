# Análise de Capturas Reais (Case Studies)

Este documento registra as capturas efetuadas em bancada utilizando o ESP32 / Bus Pirate (firmware Bruce) conectado aos pinos **RX (GPIO 5)** e **TX (GPIO 15)**.

---

## 1. Cenário Comparativo Perfeito: 41°C Chama Apagada vs. 41°C Chama Acesa

Mantendo a temperatura **rigorosamente fixa em 41°C**, isolamos exclusivamente a variável **CHAMA**:

### A. 41°C com Chama Apagada (56 bytes exatos):
```text
Hex:   55 55 55 55 55 45 54 55 45 44 51 15 14 00 45 15 45 51 45 45 54 55 55 14 15 
       44 45 14 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 15 55 55 55 55 54 55 
       55 54 51 44 14 0C

ASCII: UUUUUETUEDQ\x15\x14\x00E\x15EQEETUU\x14\x15DE\x14\x15Q\x14U\x11EDQUDED\x15Q5\x15UUUUTUUTQD\x14\x0c
```

### B. 41°C com Chuveiro Ligado / Chama Acesa (56 bytes exatos):
```text
Hex:   5D 55 55 55 55 5D 77 77 77 77 77 77 77 5D 57 55 57 57 01 55 01 55 55 5D 77 
       77 77 77 77 77 77 5D 57 55 75 55 55 5D 55 55 55 55 5D 77 77 77 77 77 77 77 
       5D 57 55 55 77 77

ASCII: ]UUUU]wwwwwww]WUWW\x01U\x01UU]wwwwwww]WUuUU]UUUU]wwwwwww]WUUww
```

---

## 2. A Descoberta da Inversão de Polaridade dos Pulsos

A análise dos histogramas de largura de pulso entre os dois estados revela:

| Estado | Duração dos Níveis ALTO (HIGH) | Duração dos Níveis BAIXO (LOW) |
| :--- | :--- | :--- |
| **Chama Apagada (41°C)** | Predominância estrita de **$360\,\mu\text{s}$** ($1\times$) | Pulsos longos de **$1070\,\mu\text{s}$** ($3\times$ base) ocorrem 55 vezes |
| **Chama Acesa (41°C)** | Pulsos longos de **$1070\,\mu\text{s}$** ocorrem **60 vezes**! | Predominância estrita de **$360\,\mu\text{s}$** ($1\times$) |

### O que isso significa:
1. **Comportamento da Linha em Repouso:** Os símbolos utilizam pulsos alargados em nível BAIXO (active-low pulse width).
2. **Comportamento em Combustão:** Quando a válvula de gás abre e a ionização confirma a chama, a modulação inverte a polaridade ativa para nível ALTO (active-high pulse width), gerando os blocos de caracteres `0x77` (`'w' = 0111 0111_b`).
3. O enquadramento tem exatamente o mesmo tamanho temporal (**56 bytes na UART = ~199.9 ms**), comprovando que a cadência do ciclo da central é rigorosamente constante.
