#!/usr/bin/env python3
"""
Bus Pirate Serial Sniffer & Analyzer for Rinnai Bus
Lê transições nos pinos G0, G25, G26 via USB Serial do Bus Pirate (firmware Bruce / Bus Pirate CLI).
"""

import sys
import time
import argparse
import serial
from decoder import decode_status_packet, format_dashboard

RINNAI_BASE_US = 370
RINNAI_SYNC_MIN_US = 2500

class BusPirateSerialAnalyzer:
    def __init__(self, port="/dev/cu.usbserial-5B090189031", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        print(f"[*] Conectando ao Bus Pirate em {self.port} ({self.baud} baud)...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        # Limpar buffers e inicializar prompt
        self.ser.write(b"\r\n\r\n")
        time.sleep(0.3)
        self.ser.read_all()
        print("[+] Conectado com sucesso!")

    def send_cmd(self, cmd: str, wait=0.3) -> str:
        self.ser.write(cmd.encode() + b"\r\n")
        time.sleep(wait)
        return self.ser.read_all().decode("utf-8", errors="replace")

    def read_pins_status(self, pins=[0, 25, 26]):
        results = {}
        for p in pins:
            out = self.send_cmd(f"read {p}", wait=0.1)
            # Parse output
            val = "HIGH (1)" if "HIGH" in out or "= 1" in out else "LOW (0)" if "LOW" in out or "= 0" in out else "UNKNOWN"
            results[p] = val
        return results

    def measure_frequency(self, pin: int, duration_ms: int = 1000):
        out = self.send_cmd(f"measure {pin} {duration_ms}", wait=(duration_ms / 1000.0) + 0.3)
        return out

    def sniff_stream(self, pin: int, duration_sec: int = 10):
        print(f"\n[*] Iniciando 'sniff' no pino {pin} por {duration_sec}s...")
        print("[*] (Pressione botões no painel frontal agora para capturar comandos!)\n")
        self.ser.write(f"sniff {pin}\r\n".encode())
        start = time.time()
        buf = ""
        try:
            while time.time() - start < duration_sec:
                raw = self.ser.read(256).decode("utf-8", errors="replace")
                if raw:
                    buf += raw
                    print(raw, end="", flush=True)
        finally:
            self.ser.write(b"\r\n")
            time.sleep(0.2)
            self.ser.read_all()
            print(f"\n[*] Sniff no pino {pin} concluído.")
        return buf

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

def main():
    parser = argparse.ArgumentParser(description="Bus Pirate Serial Rinnai Signal Analyzer")
    parser.add_argument("--port", default="/dev/cu.usbserial-5B090189031", help="Porta serial")
    parser.add_argument("--pin", type=int, default=0, choices=[0, 25, 26], help="Pino a analisar detalhadamente")
    parser.add_argument("--sniff-duration", type=int, default=10, help="Duração do sniff em segundos")
    args = parser.parse_args()

    analyzer = BusPirateSerialAnalyzer(port=args.port)
    try:
        analyzer.connect()

        print("\n--- Estado Atual dos Pinos ---")
        status = analyzer.read_pins_status([0, 25, 26])
        for p, s in status.items():
            print(f"  GPIO {p:2d}: {s}")

        print("\n--- Medição de Frequência / Bordas (1000ms cada) ---")
        for p in [0, 25, 26]:
            print(f"[+] GPIO {p}:")
            resp = analyzer.measure_frequency(p, 1000)
            for line in resp.splitlines():
                if "Rising" in line or "Falling" in line or "frequency" in line:
                    print(f"    {line.strip()}")

        analyzer.sniff_stream(args.pin, duration_sec=args.sniff_duration)

    finally:
        analyzer.close()

if __name__ == "__main__":
    main()
