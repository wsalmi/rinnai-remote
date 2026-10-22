#!/usr/bin/env python3
"""
Coletor Interativo de Amostras de Temperatura Rinnai
Permite capturar sequencialmente os quadros de 35°C a 46°C conforme o usuário
altera o painel e confirma com ENTER.
"""

import sys
import os
import time
import json
import serial

PORT = "/dev/cu.usbserial-5B090189031"
BAUD = 115200
OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "captured_temps.json")

# Sequência solicitada: 35°C até 46°C
TARGET_TEMPS = list(range(35, 47))

def init_serial():
    print(f"\n[*] Conectando ao dispositivo em {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    ser.write(b"\x03\r\n\r\n")
    time.sleep(0.05)
    ser.read_all()

    print("[*] Configurando UART passiva no pino G25 (TX em dummy 10 @ 2800 baud)...")
    ser.write(b"mode uart\r")
    time.sleep(0.05)
    ser.read_all()

    fields = ['25', '10', '2800', '8', 'N', '1', 'n']
    for f in fields:
        ser.write(f.encode() + b'\r')
        time.sleep(0.02)
    time.sleep(0.05)
    ser.read_all()

    print("[+] Modo streaming ativado.\n")
    ser.write(b"bridge\r\n")
    time.sleep(0.05)
    ser.read_all()
    return ser

def capture_frames(ser, count=3, max_seconds=3.0):
    """Captura 'count' quadros consecutivos de 63 bytes alinhados pelo preâmbulo."""
    frames = []
    buf = bytearray()
    t0 = time.time()

    # Descarta dados acumulados antes da medição
    ser.read_all()

    while len(frames) < count and (time.time() - t0) < max_seconds:
        chunk = ser.read(128)
        if chunk:
            buf.extend(chunk)
            while True:
                idx = buf.find(b"\x15\x45\x15\x51")
                if idx == -1:
                    if len(buf) > 4:
                        buf = buf[-4:]
                    break
                if len(buf) < idx + 63:
                    break
                frame = bytes(buf[idx : idx + 63])
                frames.append(frame)
                buf = buf[idx + 63 :]
                if len(frames) >= count:
                    break

    return frames

def main():
    print("=" * 65)
    print("    COLETOR INTERATIVO DE CALIBRAÇÃO DE TEMPERATURA RINNAI")
    print("=" * 65)
    print("Este script irá guiá-lo grau a grau de 35°C a 46°C.")
    print("Para cada temperatura:")
    print("  1. Ajuste no painel físico")
    print("  2. Aguarde 1 segundo para a linha estabilizar")
    print("  3. Pressione [ENTER] para capturar os quadros")
    print("=" * 65)

    try:
        ser = init_serial()
    except Exception as e:
        print(f"\n[ERRO] Não foi possível abrir a porta {PORT}: {e}")
        sys.exit(1)

    dataset = {}
    
    # Se já existir arquivo anterior, carrega para permitir retomada
    if os.path.exists(OUTPUT_FILE):
        try:
            with open(OUTPUT_FILE, "r") as f:
                dataset = json.load(f)
            print(f"[*] Carregadas {len(dataset)} amostras já existentes em {OUTPUT_FILE}")
        except Exception:
            dataset = {}

    try:
        for temp in TARGET_TEMPS:
            print("\n" + "-" * 65)
            prompt = f"👉 Ajuste o painel para [{temp}°C] e pressione [ENTER] para capturar (ou 's' para pular, 'q' para sair): "
            user_input = input(prompt).strip().lower()

            if user_input == 'q':
                print("[*] Interrompendo coleta a pedido do usuário.")
                break
            if user_input == 's':
                print(f"[*] Pulando {temp}°C.")
                continue

            print(f"[*] Lendo barramento em {temp}°C...", end="", flush=True)
            frames = capture_frames(ser, count=4, max_seconds=3.5)

            if not frames:
                print(f" ❌ Nenhum quadro capturado! Tentando novamente...")
                time.sleep(0.5)
                frames = capture_frames(ser, count=4, max_seconds=4.0)

            if not frames:
                print(f" ❌ Falha na captura de {temp}°C. Verifique se o aquecedor está ligado.")
                continue

            # Seleciona o quadro mais comum
            hex_frames = [f.hex() for f in frames]
            chosen_hex = max(set(hex_frames), key=hex_frames.count)
            chosen_bytes = bytes.fromhex(chosen_hex)

            tail_hex = chosen_bytes[-12:].hex(" ")
            print(f" ✔ OK! ({len(frames)} quadros capturados)")
            print(f"    Trailer (bytes 51..62): {tail_hex}")
            print(f"    Byte 60: 0x{chosen_bytes[-3]:02X} | Byte 62 (final): 0x{chosen_bytes[-1]:02X}")

            dataset[str(temp)] = {
                "temperature": temp,
                "frame_hex": chosen_hex,
                "tail_hex": tail_hex,
                "byte_60_hex": f"0x{chosen_bytes[-3]:02X}",
                "byte_62_hex": f"0x{chosen_bytes[-1]:02X}",
                "samples_count": len(frames),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }

            # Salva incrementalmente
            with open(OUTPUT_FILE, "w") as f:
                json.dump(dataset, f, indent=2)

    except KeyboardInterrupt:
        print("\n\n[*] Coleta finalizada pelo usuário (Ctrl+C).")
    finally:
        ser.write(b"\x03\r\n")
        time.sleep(0.1)
        ser.close()
        print(f"\n[+] Porta serial liberada.")

    print("\n" + "=" * 65)
    print(f"RESUMO DA COLETA ({len(dataset)}/{len(TARGET_TEMPS)} temperaturas salvas em {OUTPUT_FILE}):")
    print("=" * 65)
    print(f"{'Temp':<6} | {'Byte 60':<8} | {'Byte 62':<8} | {'Trailer Completo (últimos 12 bytes)'}")
    print("-" * 65)
    for t in TARGET_TEMPS:
        data = dataset.get(str(t))
        if data:
            print(f"{t:>3}°C  | {data['byte_60_hex']:<8} | {data['byte_62_hex']:<8} | {data['tail_hex']}")
        else:
            print(f"{t:>3}°C  | {'--':<8} | {'--':<8} | (não coletado)")
    print("=" * 65)

if __name__ == "__main__":
    main()
