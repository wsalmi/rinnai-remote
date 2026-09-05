#!/usr/bin/env python3
"""
Rinnai Water Heater Protocol Decoder Library & Live Monitor
"""

import sys
import os
import time
import re
import argparse
import socket
import base64
import struct
from typing import Optional, Dict, Any, List

def calculate_checksum(bytes_list: List[int]) -> int:
    """Calcula o checksum XOR dos primeiros 5 bytes."""
    chk = 0
    for b in bytes_list[:5]:
        chk ^= b
    return chk

def check_odd_parity(byte_val: int) -> bool:
    """Verifica se o bit 7 (MSB) satisfaz a paridade ímpar."""
    ones = bin(byte_val).count("1")
    return (ones % 2) != 0

def decode_status_packet(packet: bytes) -> Optional[Dict[str, Any]]:
    """
    Decodifica um pacote de 6 bytes vindo da central principal.
    Retorna dicionário com os campos ou None se for inválido.
    """
    if len(packet) != 6:
        return None

    b = list(packet)

    # Validar Checksum XOR
    expected_chk = calculate_checksum(b)
    if b[5] != expected_chk:
        return None

    # Byte 0: Origem e Prioridade
    is_main = (b[0] & 0x0F) == 0x07

    # Byte 1: Estado do Sistema
    power_on = bool(b[1] & 0x40)
    special_temp_scale = bool(b[1] & 0x08)

    # Byte 2: Chama e Temperatura
    flame_active = bool(b[2] & 0x10)
    temp_code = b[2] & 0x0F

    if temp_code == 0 and special_temp_scale:
        temperature = 35
    elif temp_code <= 11:
        temperature = 37 + temp_code
    elif temp_code == 12:
        temperature = 50
    elif temp_code == 13:
        temperature = 55
    elif temp_code == 14:
        temperature = 60
    else:
        temperature = 37 + temp_code

    # Byte 3: Potência / Exaustor
    raw_power = b[3]

    # Byte 4: Diagnóstico / Erros
    diag_code = b[4]
    has_error = (diag_code != 0x20)

    # Validar paridade ímpar de cada byte
    parity_valid = all(check_odd_parity(byte_val) for byte_val in b)

    return {
        "source": "MAIN" if is_main else "REMOTE",
        "power_on": power_on,
        "flame_active": flame_active,
        "temperature_celsius": temperature,
        "temperature_code": temp_code,
        "power_exhaust_raw": raw_power,
        "diag_code_hex": f"0x{diag_code:02X}",
        "has_error": has_error,
        "checksum_valid": True,
        "parity_valid": parity_valid,
        "raw_hex": " ".join(f"{x:02X}" for x in b)
    }

def parse_hex_packet(data_str: str) -> Optional[bytes]:
    """Extrai 6 bytes hexadecimais a partir de texto flexível."""
    cleaned = re.sub(r'0x|[^0-9A-Fa-f]', ' ', data_str).strip()
    parts = cleaned.split()
    if len(parts) == 6:
        try:
            return bytes(int(p, 16) for p in parts)
        except ValueError:
            return None
    # Tentativa de hex contínuo com 12 caracteres (6 bytes)
    continuous = re.sub(r'[^0-9A-Fa-f]', '', data_str)
    if len(continuous) == 12:
        try:
            return bytes.fromhex(continuous)
        except ValueError:
            return None
    return None

def format_dashboard(info: Dict[str, Any], stats: Dict[str, Any], source_desc: str = "", use_color: bool = True) -> str:
    """Gera visualização de terminal formatada para o modo de visão sempre ativa."""
    if use_color:
        C_RESET = "\033[0m"
        C_BOLD = "\033[1m"
        C_CYAN = "\033[1;36m"
        C_GREEN = "\033[1;32m"
        C_YELLOW = "\033[1;33m"
        C_RED = "\033[1;31m"
        C_GRAY = "\033[90m"
        C_BLUE = "\033[1;34m"
        BG_RED = "\033[1;41;37m"
    else:
        C_RESET = C_BOLD = C_CYAN = C_GREEN = C_YELLOW = C_RED = C_GRAY = C_BLUE = BG_RED = ""

    # Formatar estados
    if info["power_on"]:
        power_str = f"{C_GREEN}● LIGADO{C_RESET}"
    else:
        power_str = f"{C_YELLOW}○ STANDBY{C_RESET}"

    if info["flame_active"]:
        flame_str = f"{C_RED}🔥 CHAMA ATIVA{C_RESET}"
    else:
        flame_str = f"{C_BLUE}💨 APAGADA{C_RESET}"

    temp_str = f"{C_CYAN}{info['temperature_celsius']} °C{C_RESET} {C_GRAY}(Código: {info['temperature_code']}){C_RESET}"

    if info["has_error"]:
        diag_str = f"{BG_RED} ALERTA / ERRO ({info['diag_code_hex']}) {C_RESET}"
    else:
        diag_str = f"{C_GREEN}0x20 (Normal / Sem Falhas){C_RESET}"

    chk_str = f"{C_GREEN}OK (Válido){C_RESET}" if info["checksum_valid"] else f"{C_RED}FALHA{C_RESET}"
    parity_str = f"{C_GREEN}OK (Ímpar){C_RESET}" if info.get("parity_valid", True) else f"{C_YELLOW}AVISO (Bit 7 diferente){C_RESET}"

    uptime_s = max(time.time() - stats.get("start_time", time.time()), 0.001)
    rate = stats.get("total_packets", 0) / uptime_s

    lines = [
        f"{C_BOLD}╔══════════════════════════════════════════════════════════════╗{C_RESET}",
        f"{C_BOLD}║             RINNAI WATER HEATER - LIVE MONITOR               ║{C_RESET}",
        f"{C_BOLD}╚══════════════════════════════════════════════════════════════╝{C_RESET}",
        f"  {C_BOLD}Origem do Pacote:{C_RESET}     {info['source']} ({'Central Principal' if info['source'] == 'MAIN' else 'Painel Remoto'})",
        f"  {C_BOLD}Estado Operacional:{C_RESET}   {power_str}",
        f"  {C_BOLD}Queimador / Fogo:{C_RESET}     {flame_str}",
        f"  {C_BOLD}Temperatura Alvo:{C_RESET}     {temp_str}",
        f"  {C_BOLD}Potência / Exaustor:{C_RESET}  0x{info['power_exhaust_raw']:02X} ({info['power_exhaust_raw']})",
        f"  {C_BOLD}Diagnóstico / Erro:{C_RESET}   {diag_str}",
        f"  {C_BOLD}Integridade Dados:{C_RESET}    Checksum: {chk_str} | Paridade: {parity_str}",
        f"  {C_BOLD}Bytes Hexadecimais:{C_RESET}   {C_BOLD}{info['raw_hex']}{C_RESET}",
        f"{C_GRAY}──────────────────────────────────────────────────────────────{C_RESET}",
        f"  {C_BOLD}Pacotes Recebidos:{C_RESET}    {stats.get('total_packets', 0)} ({rate:.1f} pkt/s)",
        f"  {C_BOLD}Última Atualização:{C_RESET}   {time.strftime('%H:%M:%S')} (há {time.time() - stats.get('last_update', time.time()):.1f}s)",
        f"  {C_BOLD}Fonte de Dados:{C_RESET}       {source_desc or 'Entrada Padrão (Stdin)'}",
        f"{C_GRAY}──────────────────────────────────────────────────────────────{C_RESET}",
        f"  {C_GRAY}[Pressione Ctrl+C para encerrar o monitoramento]{C_RESET}"
    ]
    return "\n".join(lines)

def run_ws_stream(host: str, port: int, on_packet_cb):
    """Conecta ao WebSocket do Bus Pirate e decodifica continuamente."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, port))
    key = base64.b64encode(os.urandom(16)).decode('utf-8')
    req = (f'GET /ws HTTP/1.1\r\n'
           f'Host: {host}:{port}\r\n'
           f'Upgrade: websocket\r\n'
           f'Connection: Upgrade\r\n'
           f'Sec-WebSocket-Key: {key}\r\n'
           f'Sec-WebSocket-Version: 13\r\n\r\n')
    s.sendall(req.encode('utf-8'))
    resp = s.recv(4096)
    if b'101 Switching Protocols' not in resp:
        raise ConnectionError(f"Falha no handshake WebSocket: {resp}")

    # Envia comando de bridge
    payload = b"bridge\n"
    mask = os.urandom(4)
    header = bytearray([0x81, 0x80 | len(payload)])
    masked = bytearray([b ^ mask[i % 4] for i, b in enumerate(payload)])
    s.sendall(header + mask + masked)

    buffer = bytearray()
    try:
        while True:
            try:
                s.settimeout(0.5)
                data = s.recv(4096)
            except socket.timeout:
                continue
            if not data:
                break
            idx = 0
            chunks = bytearray()
            while idx < len(data):
                if idx + 1 >= len(data): break
                b1 = data[idx+1]
                masked = (b1 & 0x80) != 0
                plen = b1 & 0x7F
                idx += 2
                if plen == 126:
                    plen = struct.unpack('>H', data[idx:idx+2])[0]
                    idx += 2
                elif plen == 127:
                    plen = struct.unpack('>Q', data[idx:idx+8])[0]
                    idx += 8
                if masked:
                    mask = data[idx:idx+4]
                    idx += 4
                    pdata = bytes(b ^ mask[i%4] for i, b in enumerate(data[idx:idx+plen]))
                else:
                    pdata = data[idx:idx+plen]
                idx += plen
                chunks.extend(pdata)

            if chunks:
                buffer.extend(chunks)
                while len(buffer) >= 6:
                    pkt = bytes(buffer[:6])
                    info = decode_status_packet(pkt)
                    if info:
                        on_packet_cb(info)
                        buffer = buffer[6:]
                    else:
                        buffer = buffer[1:]
    finally:
        try:
            s.sendall(b"\x03\n")
            s.close()
        except Exception:
            pass

def generate_simulated_packet(step: int) -> bytes:
    """Gera sequência dinâmica de pacotes válidos simulando o ciclo de um aquecedor."""
    # Simula ciclo de 60 passos (~12 segundos):
    # Passos 0-10: Standby (desligado)
    # Passos 11-20: Ligado, chama acendendo, 38°C
    # Passos 21-45: Chama ativa, modulando até 42°C
    # Passos 46-55: Chama desliga, mantendo ligado
    # Passos 56-59: Volta ao Standby
    phase = step % 60
    b0 = 0x57  # MAIN central (0x7)

    if phase < 10:
        power_on = False
        flame = False
        temp_code = 5  # 42°C ajustado
        power_raw = 0x00
    elif phase < 20:
        power_on = True
        flame = False
        temp_code = 5
        power_raw = 0x10
    elif phase < 45:
        power_on = True
        flame = True
        # Pequena variação gradual de temperatura
        temp_code = 3 + ((phase - 20) // 5)
        if temp_code > 7: temp_code = 7
        power_raw = 0x40 + (phase % 16)
    elif phase < 55:
        power_on = True
        flame = False
        temp_code = 5
        power_raw = 0x05
    else:
        power_on = False
        flame = False
        temp_code = 5
        power_raw = 0x00

    b1 = 0x40 if power_on else 0x00
    # Adiciona paridade ímpar no MSB se necessário
    if (bin(b1).count("1") % 2) == 0:
        b1 |= 0x80

    b2 = (0x10 if flame else 0x00) | (temp_code & 0x0F)
    if (bin(b2).count("1") % 2) == 0:
        b2 |= 0x80

    b3 = power_raw
    if (bin(b3).count("1") % 2) == 0:
        b3 |= 0x80

    b4 = 0x20  # Sem erros
    if (bin(b4).count("1") % 2) == 0:
        b4 |= 0x80

    # Byte 0 com paridade
    if (bin(b0).count("1") % 2) == 0:
        b0 |= 0x80

    b5 = calculate_checksum([b0, b1, b2, b3, b4])
    return bytes([b0, b1, b2, b3, b4, b5])

def run_live_view(source: str = "auto", host: str = None, port: int = 80, interval: float = 0.2, use_color: bool = True):
    """Loop principal da visão sempre ativa com atualização in-place da tela."""
    stats = {
        "start_time": time.time(),
        "last_update": time.time(),
        "total_packets": 0
    }

    # Limpar terminal e posicionar cursor no topo
    print("\033[2J\033[H", end="", flush=True)

    def render_update(info: Dict[str, Any], src_desc: str):
        stats["total_packets"] += 1
        stats["last_update"] = time.time()
        output = format_dashboard(info, stats, src_desc, use_color=use_color)
        # Move o cursor para a posição inicial sem piscar
        print("\033[H" + output + "\033[J", end="", flush=True)

    try:
        if host:
            src_desc = f"Bus Pirate WebSocket (ws://{host}:{port}/ws)"
            run_ws_stream(host, port, lambda info: render_update(info, src_desc))

        elif not sys.stdin.isatty():
            # Dados direcionados via Pipe / Stdin
            src_desc = "Fluxo via Entrada Padrão (Pipe/Stdin)"
            buffer = ""
            for line in sys.stdin:
                line = line.strip()
                if not line:
                    continue
                # Tenta parsear linha como hex
                pkt = parse_hex_packet(line)
                if pkt:
                    info = decode_status_packet(pkt)
                    if info:
                        render_update(info, src_desc)
                else:
                    # Se não for linha direta, procura sequências de 6 hexadecimais
                    matches = re.findall(r'(?:[0-9A-Fa-f]{2}[\s\-:]?){6}', line)
                    for m in matches:
                        pkt = parse_hex_packet(m)
                        if pkt:
                            info = decode_status_packet(pkt)
                            if info:
                                render_update(info, src_desc)

        else:
            # Modo Simulado / Demonstração em tempo real
            src_desc = "Simulação em Tempo Real (--demo / Standalone)"
            step = 0
            while True:
                pkt = generate_simulated_packet(step)
                info = decode_status_packet(pkt)
                if info:
                    render_update(info, src_desc)
                step += 1
                time.sleep(interval)

    except KeyboardInterrupt:
        print("\n\nMonitoramento finalizado pelo usuário.")

def main():
    parser = argparse.ArgumentParser(
        description="Decodificador e Monitor em Tempo Real do Barramento Rinnai",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Exemplos de uso:
  python3 decoder.py                     # Executa teste unitário rápido
  python3 decoder.py --watch             # Inicia visão sempre ativa (modo monitor)
  python3 decoder.py -w --demo           # Visão sempre ativa com dados simulados
  python3 decoder.py -w --host 192.168.68.112  # Conecta ao Bus Pirate via WebSocket
  cat stream.log | python3 decoder.py -w       # Monitora dados vindos de pipe/stdin
  python3 decoder.py 57 40 15 00 20 22   # Decodifica pacote hexadecimal específico
"""
    )
    parser.add_argument("-w", "--watch", "--live", action="store_true", help="Ativa a visão sempre ativa com dados atualizados em tempo real")
    parser.add_argument("--demo", action="store_true", help="Força o modo de demonstração com fluxo de pacotes simulados")
    parser.add_argument("--host", default=None, help="Endereço IP do Bus Pirate para captura ao vivo via WebSocket")
    parser.add_argument("--port", type=int, default=80, help="Porta HTTP/WS do Bus Pirate (padrão: 80)")
    parser.add_argument("-i", "--interval", type=float, default=0.2, help="Intervalo de atualização em segundos para simulação (padrão: 0.2s)")
    parser.add_argument("--no-color", action="store_true", help="Desativa cores ANSI no terminal")
    parser.add_argument("packet_hex", nargs="*", help="Bytes hexadecimais do pacote para decodificar (ex: 57 40 15 00 20 22)")

    args = parser.parse_args()

    use_color = not args.no_color and sys.stdout.isatty()

    # Se foi passado um pacote posicional
    if args.packet_hex:
        raw_str = " ".join(args.packet_hex)
        pkt = parse_hex_packet(raw_str)
        if not pkt:
            print(f"Erro: Não foi possível interpretar os bytes informados: '{raw_str}'", file=sys.stderr)
            sys.exit(1)
        res = decode_status_packet(pkt)
        if not res:
            print("Erro: Pacote inválido (tamanho diferente de 6 ou checksum XOR incorreto).", file=sys.stderr)
            sys.exit(1)
        print("Resultado da decodificação:")
        for k, v in res.items():
            print(f"  {k}: {v}")
        return

    # Modo visão sempre ativa
    if args.watch:
        run_live_view(
            host=args.host,
            port=args.port,
            interval=args.interval,
            use_color=use_color
        )
        return

    # Se não foi passado --watch nem pacote posicional:
    # Executa teste padrão de decodificação para compatibilidade com os testes existentes
    test_packet = bytes([0x57, 0x40, 0x15, 0x00, 0x20, 0x57 ^ 0x40 ^ 0x15 ^ 0x00 ^ 0x20])
    res = decode_status_packet(test_packet)
    print("Teste de decodificação:")
    for k, v in res.items():
        print(f"  {k}: {v}")
    print("\nDica: Para monitoramento contínuo em tempo real, use: python3 decoder.py --watch")

if __name__ == "__main__":
    main()

