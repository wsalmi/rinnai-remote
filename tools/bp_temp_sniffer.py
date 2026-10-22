#!/usr/bin/env python3
"""
Sniffer e Decodificador em Tempo Real de Temperatura Rinnai
Conecta na UART do Bus Pirate / M5StickC (Pino RX = G25 @ 2800 baud).
"""

import sys
import time
import serial
from decoder import decode_status_packet

PORT = "/dev/cu.usbserial-5B090189031"
BAUD = 115200

def uart_to_bitstream(data: bytes) -> list:
    """Converte os bytes recebidos na UART (Start + 8 data bits LSB + Stop) em sequência de bits lógicos."""
    bits = []
    for b in data:
        # Start bit (0)
        bits.append(0)
        # 8 data bits (LSB-first)
        for i in range(8):
            bits.append((b >> i) & 1)
        # Stop bit (1)
        bits.append(1)
    return bits

def decode_rinnai_from_uart(data: bytes):
    """
    Decodifica a telemetria a partir do fluxo UART de 2800 baud.
    Calcula diferenças e mapeia para a escala de temperatura Rinnai.
    """
    pass

def main():
    print(f"[*] Conectando ao Bus Pirate em {PORT}...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(0.3)
    ser.write(b"\r\n\r\n")
    time.sleep(0.1)
    ser.read_all()

    # Configura modo UART no pino 25 a 2800 baud
    print("[*] Configurando UART no pino G25 a 2800 baud...")
    ser.write(b"mode uart\r")
    time.sleep(0.2)
    ser.read_all()

    # RX=25 (telemetria), TX=10 (pino dummy/desconectado para modo 100% passivo), Baud=2800
    fields = ['25', '10', '2800', '8', 'N', '1', 'n']
    for f in fields:
        ser.write(f.encode() + b'\r')
        time.sleep(0.1)
    time.sleep(0.3)
    ser.read_all()

    # Inicia modo bridge
    print("[*] Iniciando captura contínua...")
    ser.write(b"bridge\r\n")
    time.sleep(0.2)
    ser.read_all()

    print("\n" + "="*60)
    print("   MONITOR DE TEMPERATURA RINNAI EM TEMPO REAL")
    print("   Pressione o botão no painel para alterar a temperatura!")
    print("="*60 + "\n")

    from live_dashboard import calculate_temperature

    last_print_time = time.time()
    buffer = bytearray()

    while True:
        try:
            chunk = ser.read(128)
            if chunk:
                buffer.extend(chunk)

                if len(buffer) > 500:
                    buffer = buffer[-300:]

                # Procura alinhamento com preâmbulo de 63 bytes
                preamble = bytes.fromhex("15 45 15 51")
                idx = buffer.find(preamble)
                if idx != -1 and len(buffer) >= idx + 63:
                    frame = bytes(buffer[idx:idx+63])
                    temp = calculate_temperature(frame)

                    now = time.time()
                    if now - last_print_time > 1.0:
                        last_print_time = now
                        ts = time.strftime("%H:%M:%S")

                        if temp is not None:
                            status_temp = f"{temp}°C (Calculado)"
                        else:
                            status_temp = "Detectando..."

                        print(f"[{ts}] Temp Atual: {status_temp} | Frame final: {frame[-8:].hex(' ')}")

        except KeyboardInterrupt:
            print("\n[*] Encerrando monitor...")
            ser.write(b"\x03\r\n")
            time.sleep(0.2)
            break

    ser.close()

if __name__ == "__main__":
    main()
