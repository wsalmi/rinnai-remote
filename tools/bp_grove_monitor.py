#!/usr/bin/env python3
"""
Monitor contínuo para os pinos da porta Grove (G32 e G33).
"""

import sys
import time
import serial

PORT = "/dev/cu.usbserial-5B090189031"
BAUD = 115200

def main():
    print(f"[*] Conectando a {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.3)

    ser.write(b"\r\nmode dio\r\n")
    time.sleep(0.2)
    ser.read_all()

    ser.write(b"pullup 33\r\n")
    time.sleep(0.1)
    ser.write(b"pullup 32\r\n")
    time.sleep(0.1)
    ser.read_all()

    print("\n" + "="*60)
    print("   MONITORANDO PORTA GROVE (G32 e G33)")
    print("   Pressione os botões do painel (Power, +, -) ou abra a água!")
    print("="*60 + "\n")

    while True:
        try:
            line_parts = []
            for p in [32, 33]:
                ser.write(f"measure {p} 500\r\n".encode())
                time.sleep(0.65)
                m_resp = ser.read_all().decode("utf-8", errors="replace")

                f_str = "0.00 Hz"
                edges = "0"
                for l in m_resp.splitlines():
                    if "Rising edges:" in l:
                        edges = l.split(":")[-1].strip()
                    elif "frequency:" in l:
                        f_str = l.split(":")[-1].strip()

                ser.write(f"read {p}\r\n".encode())
                time.sleep(0.08)
                r_resp = ser.read_all().decode("utf-8", errors="replace")
                state = "HIGH" if "HIGH" in r_resp or "= 1" in r_resp else "LOW "

                line_parts.append(f"G{p}: {state} ({f_str:8s} | {edges:3s} bordas)")

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}]  " + "  |  ".join(line_parts))

            # Se houver atividade em algum pino
            for p in [32, 33]:
                if any(x in line_parts[p == 33] for x in ["kHz", " Hz"]) and "0.00 Hz" not in line_parts[p == 33]:
                    print(f"\n[!] PULSOS DETECTADOS NO G{p}! Iniciando captura de deltas...")
                    ser.write(f"sniff {p}\r\n".encode())
                    s_start = time.time()
                    while time.time() - s_start < 4:
                        raw = ser.read(256).decode("utf-8", errors="replace")
                        if raw:
                            print(raw, end="", flush=True)
                    ser.write(b"\r\n")
                    time.sleep(0.2)
                    ser.read_all()
                    print("\n[*] Fim da captura de pulsos.\n")

        except KeyboardInterrupt:
            print("\n[*] Encerrando monitoramento...")
            break

    ser.close()

if __name__ == "__main__":
    main()
