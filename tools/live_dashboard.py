#!/usr/bin/env python3
"""
Painel Virtual e Monitor em Tempo Real do Barramento Digital Rinnai
Conecta ao Bus Pirate / M5StickC Plus2 na porta serial em modo 100% passivo (Pino RX = G25).
"""

import sys
import os
import time
import signal
import serial
from typing import Optional, Dict, Any, List

PORT = "/dev/cu.usbserial-5B090189031"
BAUD = 115200


def get_frame_telemetry(frame_bytes: bytes) -> str:
    raw_bits = []
    for b in frame_bytes:
        raw_bits.append(0)
        for i in range(8):
            raw_bits.append((b >> i) & 1)
        raw_bits.append(1)
    runs = []
    curr = raw_bits[0]; cnt = 0
    for b in raw_bits:
        if b == curr: cnt += 1
        else:
            runs.append((curr, cnt))
            curr = b; cnt = 1
    runs.append((curr, cnt))
    bits = [1 if cnt >= 2 else 0 for val, cnt in runs if val == 0]
    s = ''.join(str(x) for x in bits)
    idx = s.find('1000001', 195)
    if idx == -1: idx = s.rfind('1000001')
    chunk = s[idx+7 : idx+7+8] if idx != -1 else 'none'
    tail = frame_bytes[-8:].hex(' ')
    return f"chunk={chunk} tail={tail}"

def calculate_temperature(frame_bytes: bytes) -> Optional[int]:
    """
    Decodifica analiticamente o valor exato da temperatura ajustada.
    """
    if len(frame_bytes) < 63:
        return None

    # 1. Demodulação do trem de pulsos PWM amostrado pela UART a 2800 baud
    raw_bits = []
    for b in frame_bytes:
        raw_bits.append(0)  # start bit (0)
        for i in range(8):
            raw_bits.append((b >> i) & 1)
        raw_bits.append(1)  # stop bit (1)

    runs = []
    curr = raw_bits[0]
    cnt = 0
    for b in raw_bits:
        if b == curr:
            cnt += 1
        else:
            runs.append((curr, cnt))
            curr = b
            cnt = 1
    runs.append((curr, cnt))

    # Pulsos em LOW: duração >= 2 UART bits (~714us+) = lógico 1; 1 UART bit (~357us) = lógico 0
    bits = [1 if cnt >= 2 else 0 for val, cnt in runs if val == 0]
    s = ''.join(str(x) for x in bits)

    # 2. Localização do preâmbulo/sincronismo '1000001'
    idx = s.find('1000001', 195)
    if idx == -1:
        idx = s.rfind('1000001')

    if idx != -1:
        chunk = s[idx + 7 : idx + 7 + 5]
        if len(chunk) >= 5:
            prefix = chunk[:2]
            units = int(chunk[2:5], 2)

            if prefix == '01':
                # Faixa 40..47°C (Base 40 + unidades binárias puras)
                if units == 6:
                    # 46°C vs 47°C
                    if frame_bytes[-1] == 0xC4 and frame_bytes[-3] == 0x45:
                        return 46
                    else:
                        return 47
                elif units == 7:
                    return 47
                return 40 + units
            elif prefix == '00':
                # Faixa 35..39°C (Base 36 + unidades binárias: 0=36°C, 1=37°C, 2=38°C, 3=39°C)
                if units == 0:
                    return 36
                elif units == 1:
                    return 37
                elif units == 2:
                    return 38
                elif units == 3:
                    # 35°C (Byte 62 == 0x15 modo verão) ou 39°C linear
                    return 35 if frame_bytes[-1] == 0x15 else 39
            elif prefix == '10':
                # Faixa 48 a 60°C (Base 48 com expansão superior para 56..60°C)
                if units == 0:
                    return 56 if frame_bytes[-1] == 0x31 else 48
                elif units == 1:
                    return 57 if frame_bytes[-1] == 0x03 else 49
                elif units == 2:
                    return 58 if frame_bytes[-1] == 0x34 else 50
                elif units == 3:
                    return 59 if frame_bytes[-1] == 0x11 else 51
                elif units == 4:
                    return 60 if frame_bytes[-1] == 0x03 else 52
                elif units == 5:
                    return 53
                elif units == 6:
                    return 55 if frame_bytes[-1] == 0x14 else 54
                return 60

    # 3. Discriminador secundário via bytes de estado (caso o trem de pulsos sofra ruído)
    last_byte = frame_bytes[-1]
    byte_60 = frame_bytes[-3]
    byte_61 = frame_bytes[-2]

    if last_byte == 0x15:
        if byte_60 == 0x55:
            return 45
        elif byte_60 == 0x35:
            return 44
        elif byte_60 == 0x44:
            return 41 if frame_bytes[-5] == 0x54 else 35
        return 35
    elif last_byte == 0xC4:
        if byte_60 == 0x44:
            return 36
        elif byte_60 == 0x51:
            return 43
        elif byte_60 == 0x45:
            return 46
    elif last_byte == 0x0D:
        if byte_60 == 0x11:
            return 42
        elif frame_bytes[-12] == 0x14:
            return 47
        return 38
    elif last_byte == 0x0C:
        # 40°C possui Byte 61 = 0x14; 39°C possui Byte 61 = 0x51
        return 40 if byte_61 == 0x14 else 39
    elif last_byte == 0x03:
        if byte_61 == 0x44 and byte_60 == 0x54:
            return 48
        elif byte_61 == 0x55:
            return 57
        return 60
    elif last_byte == 0x34:
        return 58
    elif last_byte == 0x11:
        return 59
    elif last_byte == 0x14:
        return 55

    return None

# Código de caracteres ANSI
C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_CYAN = "\033[1;36m"
C_GREEN = "\033[1;32m"
C_YELLOW = "\033[1;33m"
C_RED = "\033[1;31m"
C_MAGENTA = "\033[1;35m"
C_GRAY = "\033[90m"
C_BLUE = "\033[1;34m"
C_WHITE_ON_BLUE = "\033[1;37;44m"
C_WHITE_ON_GREEN = "\033[1;37;42m"
C_WHITE_ON_RED = "\033[1;37;41m"
C_WHITE_ON_GRAY = "\033[1;37;100m"

DIGITS = {
    '0': ["██████╗ ", "██╔═══██╗", "██║   ██║", "██║   ██║", "╚██████╔╝", " ╚═════╝ "],
    '1': [" ██╗    ", "███║    ", "╚██║    ", " ██║    ", " ██║    ", " ╚═╝    "],
    '2': ["██████╗ ", "╚════██╗", " █████╔╝", "██╔═══╝ ", "███████╗", "╚══════╝"],
    '3': ["██████╗ ", "╚════██╗", " █████╔╝", " ╚═══██╗", "██████╔╝", "╚═════╝ "],
    '4': ["██╗  ██╗", "██║  ██║", "███████║", "╚════██║", "     ██║", "     ╚═╝"],
    '5': ["███████╗", "██╔════╝", "███████╗", "╚════██║", "███████║", "╚══════╝"],
    '6': ["██████╗ ", "██╔════╝", "██████╗ ", "██╔═══██╗", "╚██████╔╝", " ╚═════╝ "],
    '7': ["███████╗", "╚════██║", "    ██╔╝", "   ██╔╝ ", "   ██║  ", "   ╚═╝  "],
    '8': ["██████╗ ", "██╔═══██╗", "╚█████╔╝", "██╔═══██╗", "╚██████╔╝", " ╚═════╝ "],
    '9': ["██████╗ ", "██╔═══██╗", "╚██████║", " ╚═══██║", "██████╔╝", "╚═════╝ "],
    '-': ["        ", "        ", "███████╗", "╚══════╝", "        ", "        "]
}

class RinnaiDashboard:
    def __init__(self, port: str = PORT, baud: int = BAUD):
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.current_temp: Optional[int] = None
        self.flame_active: bool = False
        self.power_on: bool = True
        self.packet_count: int = 0
        self.start_time: float = time.time()
        self.last_update_time: float = time.time()
        self.last_telemetry: str = "--"
        self.recent_events: List[str] = []
        self.running: bool = True

    def log_event(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.recent_events.append(f"[{ts}] {msg}")
        if len(self.recent_events) > 6:
            self.recent_events = self.recent_events[-6:]

    def connect(self):
        print(f"{C_CYAN}[*] Conectando ao dispositivo em {self.port}...{C_RESET}")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self.ser.write(b"\x03\r\n\r\n")
        time.sleep(0.05)
        self.ser.read_all()

        # Configura UART passiva: RX=25, TX=10 (dummy), Baud=2800, 8N1
        print(f"{C_CYAN}[*] Configurando receptor no pino G25 (TX em dummy 10)...{C_RESET}")
        self.ser.write(b"mode uart\r")
        time.sleep(0.05)
        self.ser.read_all()

        fields = ['25', '10', '2800', '8', 'N', '1', 'n']
        for f in fields:
            self.ser.write(f.encode() + b'\r')
            time.sleep(0.02)
        time.sleep(0.05)
        self.ser.read_all()

        print(f"{C_GREEN}[+] Ativando modo streaming...{C_RESET}")
        self.ser.write(b"bridge\r\n")
        time.sleep(0.05)
        self.ser.read_all()
        self.log_event("Painel inicializado - Escutando pino G25")

    def decode_frame(self, frame: bytes):
        self.packet_count += 1
        self.last_update_time = time.time()

        # Detectar presença de chama (bloco 0x77 com modulação ativa em nível alto)
        # Conforme demonstrado em captures_and_analysis.md
        count_77 = frame.count(b'\x77')
        self.flame_active = (count_77 >= 10)

        # 1. Detectar estado de energia do painel (Ligado / Standby)
        # Quando desligado no botão Power, o trailer altera assinaturas características
        # evitando detecções falsas tanto em 47°C/48°C quanto nas faixas superiores.
        def check_panel_off(f: bytes) -> bool:
            # Faixa superior (48..60°C): quando desligado, trailer termina em 0x31 (com b60=0x14) ou 0x34
            if f[-3] == 0x14 and f[-1] == 0x31:
                return True
            if f[-1] == 0x34 and f[-3] == 0x14:
                return True
            # Faixa 47°C: quando desligado, termina em 0x11 com b60=0x45 e b58=0x11
            if f[-1] == 0x11 and f[-3] == 0x45 and f[-5] == 0x11:
                return True
            # Faixa 42°C: quando desligado, termina em 0x0C com b60=0x51 e b58=0x11
            if f[-1] == 0x0C and f[-3] == 0x51 and f[-5] == 0x11:
                return True
            return False

        is_off = check_panel_off(frame)
        new_power_on = not is_off

        if new_power_on != self.power_on:
            self.power_on = new_power_on
            if not self.power_on:
                self.log_event("Painel desligado pelo usuário (Standby)")
                self.current_temp = None
            else:
                self.log_event("Painel ligado pelo usuário")

        # Telemetria bruta ao vivo
        self.last_telemetry = get_frame_telemetry(frame)
        try:
            with open("/tmp/rinnai_live.log", "a") as f_live:
                p_str = "ON" if self.power_on else "OFF"
                f_live.write(f"[{time.strftime('%H:%M:%S')}] p={p_str} t={self.current_temp} {self.last_telemetry}\n")
        except Exception:
            pass

        if not self.power_on:
            # Painel desligado: não calcula nem exibe temperaturas falsas
            return

        # 2. Decodificação analítica da temperatura (somente com painel ligado)
        detected_temp = calculate_temperature(frame)

        if detected_temp and detected_temp != self.current_temp:
            old_t = self.current_temp
            self.current_temp = detected_temp
            if old_t is not None:
                self.log_event(f"Temperatura alterada: {old_t}°C ➔ {detected_temp}°C")
            else:
                self.log_event(f"Temperatura detectada: {detected_temp}°C")

    def render(self):
        # Move cursor para topo sem limpar buffer (evita flicker)
        uptime = time.time() - self.start_time
        rate = self.packet_count / max(uptime, 0.001)

        # Banner de estado da chama
        if self.flame_active:
            flame_badge = f"{C_WHITE_ON_RED} 🔥 QUEIMADOR ACESO / CHAMA ATIVA {C_RESET}"
        else:
            flame_badge = f"{C_WHITE_ON_BLUE} 💨 STANDBY / CHAMA APAGADA {C_RESET}"

        # Status de energia
        power_badge = f"{C_WHITE_ON_GREEN} ● LIGADO {C_RESET}" if self.power_on else f"{C_WHITE_ON_GRAY} ○ DESLIGADO (STANDBY) {C_RESET}"

        # Barra gráfica e dígitos da temperatura
        if not self.power_on:
            bar_len = 25
            bar_str = "░" * bar_len
            pct_desc = "Painel desligado"
            display_desc = f"{C_GRAY}DESLIGADO (Standby){C_RESET}"
            d1, d2 = '-', '-'
            t_val = 0
        elif self.current_temp is not None:
            t_val = self.current_temp
            pct = max(0.0, min(1.0, (t_val - 35) / 25.0))
            bar_len = 25
            filled = int(round(pct * bar_len))
            bar_str = "█" * filled + "░" * (bar_len - filled)
            pct_desc = f"{int(pct*100)}% da faixa"
            display_desc = f"{t_val:02d} °C"
            t_str = f"{t_val:02d}"
            d1 = t_str[0] if t_str[0] in DIGITS else '-'
            d2 = t_str[1] if t_str[1] in DIGITS else '-'
        else:
            bar_len = 25
            bar_str = "░" * bar_len
            pct_desc = "Aguardando sinal..."
            display_desc = "-- °C (Sincronizando...)"
            d1, d2 = '-', '-'
            t_val = 0

        deg_symbol = [" °C ", "    ", "    ", "    ", "    ", "    "]

        lines = []
        lines.append("\033[H")  # Home cursor
        lines.append(f"{C_BOLD}╔══════════════════════════════════════════════════════════════════════╗{C_RESET}")
        lines.append(f"{C_BOLD}║           PAINEL DIGITAL VIRTUAL - AQUECEDOR RINNAI                  ║{C_RESET}")
        lines.append(f"{C_BOLD}║         Monitoramento Passivo em Tempo Real (Pino G25)               ║{C_RESET}")
        lines.append(f"{C_BOLD}╚══════════════════════════════════════════════════════════════════════╝{C_RESET}")
        lines.append("")
        lines.append(f"  Status: {power_badge}   Chama: {flame_badge}")
        lines.append("")
        lines.append(f"{C_GRAY}  ┌──────────────────────────────────────────────────────────────────┐{C_RESET}")
        lines.append(f"  │  {C_BOLD}TEMPERATURA PROGRAMADA NO AQUECEDOR:{C_RESET}                            │")
        lines.append(f"  │                                                                  │")

        for row in range(6):
            if not self.power_on:
                left_num = f"{C_GRAY}{DIGITS[d1][row]} {DIGITS[d2][row]}    {C_RESET}"
                if row == 1:
                    extra = f"{C_GRAY}PAINEL EM STANDBY           {C_RESET}"
                elif row == 3:
                    extra = f"{C_GRAY}[{bar_str}]{C_RESET}"
                elif row == 4:
                    extra = f"{C_GRAY}(Pressione Power no painel){C_RESET}   "
                else:
                    extra = "                                  "
            else:
                left_num = f"{C_CYAN}{DIGITS[d1][row]} {DIGITS[d2][row]}{C_YELLOW}{deg_symbol[row]}{C_RESET}"
                if row == 1:
                    extra = f"{C_BOLD}ESCALA:{C_RESET} Min 35°C | Máx 60°C        "
                elif row == 3:
                    extra = f"{C_GREEN}[{bar_str}]{C_RESET}"
                elif row == 4:
                    if self.current_temp:
                        extra = f"{C_BOLD}{C_YELLOW}{t_val:02d} °C{C_RESET} ({pct_desc})        "
                    else:
                        extra = f"{C_GRAY}(Sincronizando barramento...){C_RESET}     "
                else:
                    extra = "                                  "
            lines.append(f"  │      {left_num}   {extra}│")

        lines.append(f"  │                                                                  │")
        lines.append(f"  │   {C_BOLD}Display Físico:{C_RESET} {C_BOLD}{C_YELLOW}{display_desc}{C_RESET}                                          │")
        lines.append(f"{C_GRAY}  └──────────────────────────────────────────────────────────────────┘{C_RESET}")
        lines.append("")
        lines.append(f"  {C_BOLD}Diagnóstico da Linha:{C_RESET}")
        lines.append(f"  • Porta: {C_CYAN}{self.port}{C_RESET} | Baud: {C_CYAN}2800 (PWM){C_RESET} | Modo: {C_GREEN}100% Passivo (RX=G25){C_RESET}")
        lines.append(f"  • Pacotes Recebidos: {C_BOLD}{self.packet_count}{C_RESET} ({rate:.1f} quadros/s) | Ciclo: {C_BOLD}~225 ms{C_RESET}")
        lines.append(f"  • Linha de Botões (G26): {C_GREEN}Desconectada / Alta Impedância (Sem Risco){C_RESET}")
        lines.append(f"  • Telemetria Bruta: {C_YELLOW}{self.last_telemetry}{C_RESET}")
        lines.append("")
        lines.append(f"{C_GRAY}  ────────────────── HISTÓRICO DE EVENTOS EM TEMPO REAL ─────────────{C_RESET}")
        if not self.recent_events:
            lines.append(f"  {C_GRAY}(Aguardando interações no painel do aquecedor...){C_RESET}")
        else:
            for ev in self.recent_events[-5:]:
                lines.append(f"  • {ev}")
        lines.append(f"{C_GRAY}  ────────────────────────────────────────────────────────────────────{C_RESET}")
        lines.append(f"  {C_GRAY}[Pressione Ctrl+C a qualquer momento para sair com segurança]{C_RESET}")
        lines.append("")

        sys.stdout.write("\n".join(lines))
        sys.stdout.flush()

    def run(self):
        self.connect()
        # Limpar tela inicial
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

        buffer = bytearray()
        preamble = bytes.fromhex("15 45 15 51")
        last_render = 0

        while self.running:
            try:
                chunk = self.ser.read(128)
                if chunk:
                    buffer.extend(chunk)

                    if len(buffer) > 500:
                        buffer = buffer[-300:]

                    # Localizar quadros de 63 bytes pelo preâmbulo
                    while True:
                        idx = buffer.find(preamble)
                        if idx == -1 or len(buffer) < idx + 63:
                            break
                        
                        frame = bytes(buffer[idx:idx+63])
                        self.decode_frame(frame)
                        buffer = buffer[idx+63:]

                # Atualizar tela a cada 200 ms
                now = time.time()
                if now - last_render >= 0.2:
                    self.render()
                    last_render = now

            except KeyboardInterrupt:
                break
            except Exception as e:
                self.log_event(f"Erro: {e}")
                time.sleep(0.5)

        self.close()

    def close(self):
        print(f"\n\n{C_YELLOW}[*] Encerrando conexão e liberando o barramento...{C_RESET}")
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b"\x03\r\n")
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass
        print(f"{C_GREEN}[+] Porta serial liberada com sucesso. Até logo!{C_RESET}\n")

def main():
    dashboard = RinnaiDashboard()
    
    def sig_handler(sig, frame):
        dashboard.running = False
    
    signal.signal(signal.SIGINT, sig_handler)
    dashboard.run()

if __name__ == "__main__":
    main()
