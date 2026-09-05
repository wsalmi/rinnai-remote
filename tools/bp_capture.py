#!/usr/bin/env python3
"""
Bus Pirate Real-Time Capture & Sniffer for Rinnai Water Heater
Conecta via WebSocket ao Bus Pirate ESP32 (Bruce Firmware) e decodifica os pacotes em tempo real.
"""

import socket
import base64
import os
import time
import struct
import argparse
from decoder import decode_status_packet

def ws_connect(host: str = '192.168.68.112', port: int = 80, path: str = '/ws'):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, port))
    key = base64.b64encode(os.urandom(16)).decode('utf-8')
    req = (f'GET {path} HTTP/1.1\r\n'
           f'Host: {host}:{port}\r\n'
           f'Upgrade: websocket\r\n'
           f'Connection: Upgrade\r\n'
           f'Sec-WebSocket-Key: {key}\r\n'
           f'Sec-WebSocket-Version: 13\r\n\r\n')
    s.sendall(req.encode('utf-8'))
    resp = s.recv(4096)
    if b'101 Switching Protocols' not in resp:
        raise ConnectionError(f"Falha no handshake WebSocket: {resp}")
    return s

def ws_send(s, text):
    payload = text.encode('utf-8') if isinstance(text, str) else text
    mask = os.urandom(4)
    if len(payload) < 126:
        header = bytearray([0x81, 0x80 | len(payload)])
    else:
        header = bytearray([0x81, 0x80 | 126]) + struct.pack('>H', len(payload))
    masked = bytearray([b ^ mask[i % 4] for i, b in enumerate(payload)])
    s.sendall(header + mask + masked)

def read_ws_frames(s, timeout=1.0):
    s.settimeout(timeout)
    chunks = bytearray()
    try:
        data = s.recv(4096)
        if not data:
            return chunks
        idx = 0
        while idx < len(data):
            b0 = data[idx]
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
    except socket.timeout:
        pass
    except Exception as e:
        print(f"[WS Error]: {e}")
    return chunks

def main():
    parser = argparse.ArgumentParser(description="Rinnai Bus Pirate Real-time Sniffer")
    parser.add_argument("--host", default="192.168.68.112", help="IP do Bus Pirate na rede")
    parser.add_argument("--port", type=int, default=80, help="Porta HTTP/WS do Bus Pirate")
    args = parser.parse_args()

    print(f"[*] Conectando ao Bus Pirate em ws://{args.host}:{args.port}/ws...")
    try:
        s = ws_connect(args.host, args.port)
        print("[+] Conectado com sucesso!")
    except Exception as e:
        print(f"[-] Erro ao conectar: {e}")
        return

    # Inicia captura em modo bridge
    print("[*] Iniciando monitoramento em modo Bridge...")
    ws_send(s, "bridge\n")

    buffer = bytearray()
    try:
        while True:
            chunk = read_ws_frames(s, 0.2)
            if chunk:
                buffer.extend(chunk)
                # Procura por delimitações de quadros (pausas ou padrões)
                while len(buffer) >= 6:
                    pkt = buffer[:6]
                    info = decode_status_packet(bytes(pkt))
                    if info:
                        print(f"\n[RINNAI STATUS] {info['raw_hex']}")
                        print(f"  Power: {'LIGADO' if info['power_on'] else 'STANDBY'} | "
                              f"Chama: {'ATIVA' if info['flame_active'] else 'APAGADA'} | "
                              f"Temp: {info['temperature_celsius']}°C | "
                              f"Diag: {info['diag_code_hex']}")
                        buffer = buffer[6:]
                    else:
                        buffer = buffer[1:]
    except KeyboardInterrupt:
        print("\n[*] Encerrando monitoramento...")
        ws_send(s, "\x03\n")
        time.sleep(0.2)
        s.close()

if __name__ == "__main__":
    main()
