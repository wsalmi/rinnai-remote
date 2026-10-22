#!/usr/bin/env python3
"""
Monitor contínuo para o Bus Pirate / M5StickC (Bruce Firmware).
Monitora G25 e G26 simultaneamente mantendo a porta serial persistente.
"""

import sys
import time
import serial

PORT = "/dev/cu.usbserial-5B090189031"
BAUD = 115200

def main():
    print(f"[*] Abrindo conexão persistente com {PORT} a {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.5)

    # Entra em modo DIO
    ser.write(b"mode dio\r\n")
    time.sleep(0.3)
    ser.read_all()

    # Ativa pullup interno no pino 25
    ser.write(b"pullup 25\r\n")
    time.sleep(0.1)
    ser.read_all()

    print("\n" + "="*60)
    print("   MONITORAMENTO DUAL (G25 e G26) - BARRAMENTO RINNAI")
    print("="*60)
    print("Monitorando transmissões de telemetria e comandos...\n")

    while True:
        try:
            results = {}
            for p in [25, 26]:
                # Mede frequencia
                ser.write(f"measure {p} 600\r\n".encode())
                time.sleep(0.75)
                resp = ser.read_all().decode("utf-8", errors="replace")

                freq_str = "0.00 Hz"
                edges_str = "0"
                for line in resp.splitlines():
                    if "Rising edges:" in line:
                        edges_str = line.split(":")[-1].strip()
                    elif "frequency:" in line:
                        freq_str = line.split(":")[-1].strip()

                # Le nivel
                ser.write(f"read {p}\r\n".encode())
                time.sleep(0.1)
                r_resp = ser.read_all().decode("utf-8", errors="replace")
                state = "HIGH" if "HIGH" in r_resp or "= 1" in r_resp else "LOW"
                results[p] = (state, edges_str, freq_str)

            ts = time.strftime("%H:%M:%S")
            s25, e25, f25 = results[25]
            s26, e26, f26 = results[26]
            print(f"[{ts}] G25: {s25:4s} ({f25:8s}) | G26: {s26:4s} ({f26:8s})")

            # Verifica se algum pino teve atividade
            for p in [25, 26]:
                try:
                    f_val = float(results[p][2].replace("Hz", "").strip())
                    if f_val > 50:
                        print(f"\n[!] ATIVIDADE NO PINO G{p} ({f_val} Hz)! Sniffing...")
                        ser.write(f"sniff {p}\r\n".encode())
                        s_start = time.time()
                        while time.time() - s_start < 4:
                            c = ser.read(256).decode("utf-8", errors="replace")
                            if c:
                                print(c, end="", flush=True)
                        ser.write(b"\r\n")
                        time.sleep(0.2)
                        ser.read_all()
                        print("\n[*] Fim do sniff.\n")
                except ValueError:
                    pass

        except KeyboardInterrupt:
            print("\n[*] Encerrando monitoramento...")
            break

    ser.close()

if __name__ == "__main__":
    main()
