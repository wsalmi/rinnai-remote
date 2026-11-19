#!/usr/bin/env python3
"""
Coletor Interativo de Calibração Full-Range (35°C a 43°C) para M5NanoC6.
Lê linhas [STATUS] e [FRAME] da serial USB CDC do M5NanoC6, salva em JSON
estruturado e exibe tabela comparativa bit-a-bit de B3 (e B1/B2).
"""

import argparse
import glob
import json
import os
import re
import sys
import time
from collections import Counter
from typing import Any, Dict, List, Optional, Tuple

STATUS_RE = re.compile(
    r"\[STATUS\]\s+ZB=(?P<zb>\S+)\s+\|\s+RxPin=(?P<rx_pin>\S+)\s+\|\s+Online=(?P<online>\S+)\s+\|\s+"
    r"Power=(?P<power>\S+)\s+\|\s+Chama=(?P<chama>\S+)\s+\|\s+Temp=(?P<temp>\d+)°C\s+\|\s+"
    r"Alvo=(?P<alvo>\d+)°C\s+\|\s+Pkts=(?P<pkts>\d+)\s+\|\s+RxBytes=(?P<rx_bytes>\d+).*?"
    r"G1=\d+(?:\(E=(?P<g1_edges>\d+),[^)]*\))?.*?\|\s+TailHEX=\[(?P<tail_hex>.*?)\]\s+\|\s+LastHEX=\[(?P<last_hex>.*?)\]"
)

FRAME_RE = re.compile(
    r"\[FRAME\]\s+len=(?P<len>\d+)\s+HEX=\[(?P<hex>[0-9A-Fa-f ]+)\]"
)


def detect_serial_port() -> Optional[str]:
    for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def parse_status_line(line: str) -> Optional[Dict[str, Any]]:
    m = STATUS_RE.search(line)
    if not m:
        return None
    g1_edges_raw = m.group("g1_edges")
    return {
        "zb": m.group("zb"),
        "rx_pin": m.group("rx_pin"),
        "online": m.group("online"),
        "power": m.group("power"),
        "chama": m.group("chama"),
        "temp": int(m.group("temp")),
        "alvo": int(m.group("alvo")),
        "pkts": int(m.group("pkts")),
        "rx_bytes": int(m.group("rx_bytes")),
        "g1_edges": int(g1_edges_raw) if g1_edges_raw is not None else None,
        "tail_hex": m.group("tail_hex").strip(),
        "last_hex": m.group("last_hex").strip(),
    }


def parse_frame_line(line: str) -> Optional[Tuple[int, bytes, str]]:
    m = FRAME_RE.search(line)
    if not m:
        return None
    frame_len = int(m.group("len"))
    hex_str = m.group("hex").strip()
    if frame_len == 0 or not hex_str or hex_str == "--":
        return None
    try:
        frame_bytes = bytes.fromhex(hex_str)
    except ValueError:
        return None
    norm_hex = " ".join(f"{b:02X}" for b in frame_bytes)
    return frame_len, frame_bytes, norm_hex


def demodulate_uart_low_bits(frame_bytes: bytes) -> str:
    """
    Espelha RinnaiFrameParser::demodulateUartToBits de rinnai_uart_decoder.h:
    start bit 0, 8 data bits LSB-first, stop bit 1; runs de 0 com cnt >= 2 -> '1', senão '0'.
    """
    if not frame_bytes:
        return ""
    raw_bits: List[int] = []
    for b in frame_bytes:
        raw_bits.append(0)
        for bit in range(8):
            raw_bits.append((b >> bit) & 0x01)
        raw_bits.append(1)

    bits_out: List[str] = []
    curr = raw_bits[0]
    cnt = 0
    for val in raw_bits:
        if val == curr:
            cnt += 1
        else:
            if curr == 0:
                bits_out.append("1" if cnt >= 2 else "0")
            curr = val
            cnt = 1
    if curr == 0:
        bits_out.append("1" if cnt >= 2 else "0")
    return "".join(bits_out)


def demodulate_64bit_bytes(burst_bytes: bytes) -> Tuple[str, str]:
    """
    Demodula um sub-bloco até o delimitador de fim de quadro (HIGH >= 2 ou LOW >= 4)
    e retorna (pkt8_hex, bitstring_64) com os primeiros 8 bytes MSB-first.
    """
    if not burst_bytes:
        return "", ""
    raw_bits: List[int] = []
    for b in burst_bytes:
        raw_bits.append(0)
        for bit in range(8):
            raw_bits.append((b >> bit) & 0x01)
        raw_bits.append(1)

    runs: List[Tuple[int, int]] = []
    curr = raw_bits[0]
    cnt = 0
    for val in raw_bits:
        if val == curr:
            cnt += 1
        else:
            runs.append((curr, cnt))
            curr = val
            cnt = 1
    runs.append((curr, cnt))

    bits: List[str] = []
    for val, run_len in runs:
        if val == 1 and run_len >= 2:
            break
        if val == 0:
            if run_len >= 4:
                break
            bits.append("1" if run_len >= 2 else "0")

    bitstr = "".join(bits)
    byte_vals = [int(bitstr[i:i + 8], 2) for i in range(0, min(len(bitstr), 64) - 7, 8)]
    pkt8_hex = " ".join(f"{v:02X}" for v in byte_vals)
    return pkt8_hex, bitstr


def analyze_frame_bytes(frame_bytes: bytes) -> Dict[str, Any]:
    bursts = [b for b in frame_bytes.split(b"\x00") if len(b) > 0]
    bursts_hex = [" ".join(f"{x:02X}" for x in b) for b in bursts]
    b1_bits = demodulate_uart_low_bits(bursts[1]) if len(bursts) > 1 else ""
    b2_bits = demodulate_uart_low_bits(bursts[2]) if len(bursts) > 2 else ""
    b3_bytes = bursts[-1] if bursts else b""
    b3_hex = " ".join(f"{x:02X}" for x in b3_bytes)
    b3_bits = demodulate_uart_low_bits(b3_bytes)
    b3_pkt8_hex, _ = demodulate_64bit_bytes(b3_bytes)

    sync_idx = b3_bits.find("1000001")
    b3_code5 = ""
    b3_after_sync = ""
    has_1011100 = False
    if sync_idx != -1:
        after = b3_bits[sync_idx + 7:]
        b3_after_sync = after
        b3_code5 = after[:5] if len(after) >= 5 else after
        has_1011100 = "1011100" in b3_bits[:sync_idx]

    return {
        "frame_len": len(frame_bytes),
        "bursts_hex": bursts_hex,
        "b1_bits": b1_bits,
        "b2_bits": b2_bits,
        "b3_hex": b3_hex,
        "b3_bits": b3_bits,
        "b3_pkt8_hex": b3_pkt8_hex,
        "b3_sync_idx": sync_idx,
        "b3_code5": b3_code5,
        "b3_after_sync": b3_after_sync,
        "has_1011100": has_1011100,
        "count_0x77": frame_bytes.count(0x77),
    }


def load_dataset(path: str, min_temp: int, max_temp: int, port: str) -> Dict[str, Any]:
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict):
                data.setdefault("meta", {})
                data.setdefault("idle", {})
                data.setdefault("flame", {})
                data.setdefault("power_off", {})
                return data
        except Exception as e:
            print(f"[!] Aviso: não foi possível carregar {path} ({e}); iniciando novo.")
    return {
        "meta": {
            "min_temp": min_temp,
            "max_temp": max_temp,
            "port": port,
            "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "idle": {},
        "flame": {},
        "power_off": {},
    }


def save_dataset(path: str, data: Dict[str, Any], min_temp: int, max_temp: int, port: str) -> None:
    data["meta"] = {
        "min_temp": min_temp,
        "max_temp": max_temp,
        "port": port,
        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def drain_serial_buffer(ser: Any, drain_sec: float = 0.25) -> None:
    ser.reset_input_buffer()
    deadline = time.time() + drain_sec
    while time.time() < deadline:
        waiting = getattr(ser, "in_waiting", 0)
        chunk = ser.read(waiting if waiting else 256)
        if not chunk:
            time.sleep(0.02)


def capture_samples(ser: Any, count: int, timeout_s: float = 15.0) -> List[Dict[str, Any]]:
    drain_serial_buffer(ser, 0.25)
    samples: List[Dict[str, Any]] = []
    last_status: Optional[Dict[str, Any]] = None
    deadline = time.time() + timeout_s

    sys.stdout.write(f"    Aguardando {count} amostra(s) na serial ")
    sys.stdout.flush()

    while len(samples) < count and time.time() < deadline:
        raw = ser.readline()
        if not raw:
            sys.stdout.write(".")
            sys.stdout.flush()
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue

        st = parse_status_line(line)
        if st is not None:
            last_status = st
            continue

        fr = parse_frame_line(line)
        if fr is not None:
            frame_len, frame_bytes, norm_hex = fr
            samples.append({
                "frame_len": frame_len,
                "frame_hex": norm_hex,
                "status": last_status.copy() if last_status else None,
                "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            })
            sys.stdout.write(f" [{len(samples)}/{count}]")
            sys.stdout.flush()

    sys.stdout.write("\n")
    sys.stdout.flush()
    return samples


def build_entry(expected_temp: Optional[int], mode: str, samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    hex_counts = Counter(s["frame_hex"] for s in samples)
    primary_hex = hex_counts.most_common(1)[0][0]
    primary_bytes = bytes.fromhex(primary_hex)
    analysis = analyze_frame_bytes(primary_bytes)

    fw_status: Optional[Dict[str, Any]] = None
    for s in reversed(samples):
        if s["frame_hex"] == primary_hex and s.get("status"):
            fw_status = s["status"]
            break
    if fw_status is None:
        for s in reversed(samples):
            if s.get("status"):
                fw_status = s["status"]
                break

    return {
        "expected_temp": expected_temp,
        "mode": mode,
        "samples": samples,
        "primary_frame_hex": primary_hex,
        **analysis,
        "fw_status": fw_status or {},
    }


def print_live_summary(entry: Dict[str, Any], expected_temp: Optional[int], mode: str) -> None:
    fw = entry.get("fw_status") or {}
    fw_temp = fw.get("temp")
    fw_chama = fw.get("chama", "?")
    fw_power = fw.get("power", "?")

    temp_match = (expected_temp is None) or (fw_temp == expected_temp)
    if mode == "flame":
        mode_match = (fw_chama == "ATIVA" and fw_power == "HEAT")
    elif mode == "idle":
        mode_match = (fw_chama == "IDLE" and fw_power == "OFF")
    else:
        mode_match = (fw_chama == "IDLE" and fw_power == "OFF")

    verdict = "OK" if (temp_match and mode_match) else "DIVERGENTE"
    sync_slice = ""
    if entry["b3_sync_idx"] != -1:
        sync_slice = "1000001+" + entry["b3_after_sync"][:16]
    else:
        sync_slice = f"(sem sync) {entry['b3_bits'][:23]}"

    print(f"    Len={entry['frame_len']} | B3=[{entry['b3_hex']}]")
    print(f"    B3 pkt8=[{entry.get('b3_pkt8_hex') or '--'}] | bits={sync_slice} | code5={entry['b3_code5'] or '--'} | 1011100={entry['has_1011100']}")
    print(
        f"    FW: Online={fw.get('online', '?')} | Pkts={fw.get('pkts', '?')} | RxBytes={fw.get('rx_bytes', '?')} "
        f"| Temp={fw_temp}°C (esp={expected_temp if expected_temp is not None else '--'}°C) "
        f"| Power={fw_power} | Chama={fw_chama} -> [{verdict}]"
    )


def print_summary_table(data: Dict[str, Any], min_temp: int, max_temp: int) -> None:
    print("\n" + "=" * 124)
    print("RESUMO COMPARATIVO (35°C a 43°C)")
    print("=" * 124)
    header = (
        f"{'Temp':<5} | {'Mode':<9} | {'Len':<3} | {'FW Temp':<7} | {'FW Chama':<8} | "
        f"{'Code5':<6} | {'1011100':<7} | {'B3 8-Bytes (64b)':<24} | {'B3 HEX'}"
    )
    print(header)
    print("-" * 124)

    for mode in ("idle", "flame"):
        bucket = data.get(mode, {})
        for t in range(min_temp, max_temp + 1):
            entry = bucket.get(str(t))
            if not entry:
                continue
            fw = entry.get("fw_status") or {}
            fw_t = f"{fw['temp']}°C" if "temp" in fw else "--"
            fw_c = fw.get("chama", "--")
            c5 = entry.get("b3_code5") or "--"
            mk = "SIM" if entry.get("has_1011100") else "NAO"
            p8 = entry.get("b3_pkt8_hex") or "--"
            print(
                f"{t:>3}°C | {mode:<9} | {entry.get('frame_len', 0):<3} | {fw_t:<7} | {fw_c:<8} | "
                f"{c5:<6} | {mk:<7} | {p8:<24} | {entry.get('b3_hex', '')}"
            )

    off_bucket = data.get("power_off", {})
    for key, entry in off_bucket.items():
        fw = entry.get("fw_status") or {}
        fw_t = f"{fw['temp']}°C" if "temp" in fw else "--"
        fw_c = fw.get("chama", "--")
        c5 = entry.get("b3_code5") or "--"
        mk = "SIM" if entry.get("has_1011100") else "NAO"
        p8 = entry.get("b3_pkt8_hex") or "--"
        print(
            f"{key:<5} | {'power_off':<9} | {entry.get('frame_len', 0):<3} | {fw_t:<7} | {fw_c:<8} | "
            f"{c5:<6} | {mk:<7} | {p8:<24} | {entry.get('b3_hex', '')}"
        )
    print("=" * 124)


def check_stale_or_offline(samples: List[Dict[str, Any]], entry: Dict[str, Any], prev_pkts: Optional[int]) -> Optional[str]:
    fw = entry.get("fw_status") or {}
    online = fw.get("online")
    pkts_last = fw.get("pkts")
    pkts_first = None
    if samples:
        s0_st = samples[0].get("status") or {}
        pkts_first = s0_st.get("pkts")

    reasons: List[str] = []
    if online == "NO":
        reasons.append("M5NanoC6 reportando Online=NO (sem novos quadros da central)")
    if len(samples) >= 2 and pkts_first is not None and pkts_last is not None and pkts_last <= pkts_first:
        reasons.append(f"Pkts congelado entre amostras ({pkts_first} -> {pkts_last})")
    if prev_pkts is not None and pkts_last is not None and pkts_last <= prev_pkts:
        reasons.append(f"Pkts não avançou desde a etapa anterior ({prev_pkts} -> {pkts_last})")

    if not reasons:
        return None
    g1_e = fw.get("g1_edges")
    g1_str = str(g1_e) if g1_e is not None else "?"
    return (
        " / ".join(reasons)
        + f"\n      Online={online} | Pkts={pkts_last} | RxBytes={fw.get('rx_bytes', '?')} | G1_edges={g1_str} | LastHEX=[{fw.get('last_hex', '--')}]"
    )


def run_sweep_round(
    ser: Any,
    data: Dict[str, Any],
    mode: str,
    min_temp: int,
    max_temp: int,
    samples_count: int,
    output_path: str,
    port: str,
) -> bool:
    """Executa uma varredura min..max para 'idle' ou 'flame'. Retorna False se usuário pediu 'q'."""
    label = "Chama DESLIGADA (idle)" if mode == "idle" else "Chama ACESA (flame)"
    print("\n" + "#" * 72)
    print(f"  RODADA: {label} ({min_temp}°C -> {max_temp}°C)")
    print("#" * 72)
    if mode == "flame":
        ans = input("👉 Abra o registro de água quente para acender a chama e pressione [ENTER] (ou 's' para pular rodada, 'q' sair): ").strip().lower()
        if ans == "q":
            return False
        if ans == "s":
            print("[*] Rodada chama acesa pulada.")
            return True

    prev_hex: Optional[str] = None
    prev_temp: Optional[int] = None
    prev_pkts: Optional[int] = None

    for temp in range(min_temp, max_temp + 1):
        print(f"\n--- [{mode.upper()}] Temperatura alvo: {temp}°C ---")
        ans = input(f"👉 Ajuste o painel para [{temp}°C], aguarde 1s e pressione [ENTER] ('s' pular, 'q' sair): ").strip().lower()
        if ans == "q":
            return False
        if ans == "s":
            print(f"[*] Pulando {temp}°C ({mode}).")
            continue

        while True:
            samples = capture_samples(ser, samples_count)
            if not samples:
                retry = input("    ❌ Timeout sem [FRAME] válido na serial. [ENTER] tentar novamente, 's' pular, 'q' sair: ").strip().lower()
                if retry == "q":
                    return False
                if retry == "s":
                    samples = []
                    break
                continue

            entry = build_entry(temp, mode, samples)
            stale_msg = check_stale_or_offline(samples, entry, prev_pkts)
            if stale_msg:
                print(f"    ⚠️ ALERTA: {stale_msg}")
                choice = input("    [ENTER] tentar novamente, 'k' salvar mesmo assim, 's' pular, 'q' sair: ").strip().lower()
                if choice == "q":
                    return False
                if choice == "":
                    continue
                if choice == "s":
                    entry = None
                    break
            elif prev_hex is not None and prev_temp is not None and temp != prev_temp and entry["primary_frame_hex"] == prev_hex:
                print("    ⚠️ Quadro idêntico à temperatura anterior!")
                choice = input("    [ENTER] recapturar, 'k' manter mesmo assim, 's' pular: ").strip().lower()
                if choice == "":
                    continue
                if choice == "s":
                    entry = None
                    break
            break

        if not samples or entry is None:
            continue

        prev_hex = entry["primary_frame_hex"]
        prev_temp = temp
        if entry["fw_status"].get("pkts") is not None:
            prev_pkts = entry["fw_status"]["pkts"]
        data[mode][str(temp)] = entry
        save_dataset(output_path, data, min_temp, max_temp, port)
        print_live_summary(entry, temp, mode)

    return True


def run_power_off_round(
    ser: Any,
    data: Dict[str, Any],
    min_temp: int,
    max_temp: int,
    samples_count: int,
    output_path: str,
    port: str,
) -> None:
    print("\n" + "#" * 72)
    print("  RODADA OPCIONAL: Painel DESLIGADO no botão Power (power_off)")
    print("#" * 72)
    ans = input("👉 Desligue o painel pelo botão Power e pressione [ENTER] (ou 's' para pular): ").strip().lower()
    if ans in ("s", "q"):
        print("[*] Rodada power_off pulada.")
        return

    while True:
        samples = capture_samples(ser, samples_count)
        if not samples:
            retry = input("    ❌ Timeout sem [FRAME] válido. [ENTER] tentar novamente, 's' pular: ").strip().lower()
            if retry == "s":
                return
            continue
        entry = build_entry(None, "power_off", samples)
        stale_msg = check_stale_or_offline(samples, entry, None)
        if stale_msg:
            print(f"    ⚠️ ALERTA: {stale_msg}")
            choice = input("    [ENTER] tentar novamente, 'k' salvar mesmo assim, 's' pular: ").strip().lower()
            if choice == "":
                continue
            if choice == "s":
                return
        break

    data["power_off"]["off"] = entry
    save_dataset(output_path, data, min_temp, max_temp, port)
    print_live_summary(entry, None, "power_off")


def run_self_test() -> int:
    cases = [
        (
            "40_IDLE",
            "[STATUS] ZB=OK | RxPin=G1 | Online=YES | Power=OFF | Chama=IDLE | Temp=40°C | Alvo=40°C | Pkts=226 | RxBytes=16112 | Burst=37 | G1=0(E=148550,553us) | G2=0(E=12636,45817us) | TailHEX=[14 55 11 54 51 14 11 45 44 54 55 54] | LastHEX=[45 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35]",
            "[FRAME] len=71 HEX=[15 45 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 00 45 15 55 55 15 51 11 45 55 51 54 51 44 55 44 11 00 00 51 15 55 55 55 55 55 45 54 55 45 44 51 15 14 00 00 55 45 15 55 14 55 11 54 51 14 11 45 44 54 55 54 03 00]",
            40, "OFF", "IDLE", "01000", False, "01 08 28 1C 25 65 A0 10", 16112, 148550,
        ),
        (
            "40_FLAME",
            "[STATUS] ZB=OK | RxPin=G1 | Online=YES | Power=HEAT | Chama=ATIVA | Temp=40°C | Alvo=40°C | Pkts=351 | RxBytes=25011 | Burst=36 | G1=0(E=214170,552us) | G2=0 | TailHEX=[14 00 00 55 14 44 51 15 51 15 51 15] | LastHEX=[55 55 45 55 44 51 14 11 45 15 44 11 45 14 11 00]",
            "[FRAME] len=73 HEX=[15 45 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 00 45 15 55 55 45 55 44 51 14 11 45 15 44 11 45 14 11 00 00 51 15 55 55 55 55 55 45 54 55 45 44 51 15 14 00 00 55 14 44 51 15 51 15 51 15 44 15 44 45 15 44 45 44 03 00]",
            40, "HEAT", "ATIVA", "01000", True, "05 C8 28 28 38 71 0E 2C", 25011, 214170,
        ),
        (
            "41_FLAME",
            "[STATUS] ZB=OK | RxPin=G1 | Online=YES | Power=HEAT | Chama=ATIVA | Temp=41°C | Alvo=41°C | Pkts=502 | RxBytes=35778 | Burst=36 | G1=0(E=293555,547us) | G2=0 | TailHEX=[34 00 55 14 44 51 15 51 54 45 15 44] | LastHEX=[00 51 15 55 55 55 55 55 45 54 55 55 15 45 44 34]",
            "[FRAME] len=72 HEX=[15 45 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 00 45 15 55 55 45 51 11 45 51 11 45 15 14 11 51 14 11 00 00 51 15 55 55 55 55 55 45 54 55 55 15 45 44 34 00 55 14 44 51 15 51 54 45 15 44 15 44 11 14 51 54 44 0D 00]",
            41, "HEAT", "ATIVA", "01001", True, "05 C8 29 04 38 73 A9 18", 35778, 293555,
        ),
        (
            "37_FLAME",
            "[STATUS] ZB=OK | RxPin=G1 | Online=YES | Power=HEAT | Chama=ATIVA | Temp=37°C | Alvo=37°C | Pkts=455 | RxBytes=32891 | Burst=36 | G1=0 | G2=0 | TailHEX=[54 44 44 44 14 45 44 14 11 51 00 00] | LastHEX=[45 15 55 55 54 44 45 45 51 11 45 15 45 55 54 C5]",
            "[FRAME] len=72 HEX=[15 45 15 51 14 55 11 45 44 51 55 44 45 44 15 51 35 00 45 15 55 55 54 44 45 45 51 11 45 15 45 55 54 C5 00 51 15 55 55 45 51 55 55 44 55 55 14 54 55 14 00 00 55 14 44 51 15 45 54 45 54 44 44 44 14 45 44 14 11 51 00 00]",
            37, "HEAT", "ATIVA", "00101", True, "05 C8 25 05 1B 6A 5A B4", 32891, None,
        ),
    ]

    for name, status_line, frame_line, exp_temp, exp_power, exp_chama, exp_code5, exp_marker, exp_pkt8, exp_rxb, exp_edges in cases:
        st = parse_status_line(status_line)
        assert st is not None, f"{name}: falha ao parsear STATUS"
        assert st["temp"] == exp_temp, f"{name}: temp={st['temp']}"
        assert st["power"] == exp_power, f"{name}: power={st['power']}"
        assert st["chama"] == exp_chama, f"{name}: chama={st['chama']}"
        assert st["rx_bytes"] == exp_rxb, f"{name}: rx_bytes={st['rx_bytes']}"
        assert st["g1_edges"] == exp_edges, f"{name}: g1_edges={st['g1_edges']}"

        fr = parse_frame_line(frame_line)
        assert fr is not None, f"{name}: falha ao parsear FRAME"
        flen, fbytes, _ = fr
        assert flen == len(fbytes), f"{name}: len mismatch {flen} != {len(fbytes)}"

        an = analyze_frame_bytes(fbytes)
        assert an["b3_code5"] == exp_code5, f"{name}: code5={an['b3_code5']} != {exp_code5}"
        assert an["has_1011100"] is exp_marker, f"{name}: has_1011100={an['has_1011100']} != {exp_marker}"
        assert an["b3_pkt8_hex"] == exp_pkt8, f"{name}: pkt8={an['b3_pkt8_hex']} != {exp_pkt8}"
        print(f"[PASS] {name}: len={flen}, pkt8=[{an['b3_pkt8_hex']}], code5={an['b3_code5']}, 1011100={an['has_1011100']}, bursts={len(an['bursts_hex'])}")

    assert parse_frame_line("[FRAME] len=0 HEX=[--]") is None
    print("[PASS] empty_frame_ignored")
    print("SELF-TEST OK (all assertions passed)")
    return 0


def main() -> None:
    default_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captured_nanoc6_35_43.json")
    ap = argparse.ArgumentParser(description="Coletor interativo full-range (35°C a 43°C) para M5NanoC6")
    ap.add_argument("--port", default=None, help="Porta serial (padrão: autodetecta /dev/cu.usbmodem*)")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate serial (padrão: 115200)")
    ap.add_argument("--min", dest="min_temp", type=int, default=35, help="Temperatura mínima (padrão: 35)")
    ap.add_argument("--max", dest="max_temp", type=int, default=43, help="Temperatura máxima (padrão: 43)")
    ap.add_argument("--mode", choices=["all", "idle", "flame", "off"], default="all", help="Modo de coleta (padrão: all)")
    ap.add_argument("--samples", type=int, default=2, help="Amostras [FRAME] por etapa (padrão: 2)")
    ap.add_argument("--output", default=default_out, help=f"Arquivo JSON de saída (padrão: {default_out})")
    ap.add_argument("--self-test", action="store_true", help="Executa testes internos de parsing e demodulação")
    args = ap.parse_args()

    if args.self_test:
        sys.exit(run_self_test())

    try:
        import serial  # type: ignore
    except ImportError:
        print("[ERRO] Biblioteca 'pyserial' não encontrada. Execute via ./run_nanoc6_collector.sh")
        sys.exit(1)

    port = args.port or detect_serial_port()
    if not port:
        print("[ERRO] Nenhuma porta /dev/cu.usbmodem* ou /dev/cu.usbserial* encontrada. Conecte o M5NanoC6 ou passe --port.")
        sys.exit(1)

    print("=" * 72)
    print(f"  COLETOR INTERATIVO M5NanoC6 ({args.min_temp}°C a {args.max_temp}°C)")
    print("=" * 72)
    print(f"Porta: {port} @ {args.baud} | Modo: {args.mode} | Amostras/etapa: {args.samples}")
    print(f"Saída: {args.output}")

    data = load_dataset(args.output, args.min_temp, args.max_temp, port)

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = args.baud
    ser.timeout = 0.5
    ser.dsrdtr = False
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    try:
        ser.open()
    except Exception as e:
        print(f"[ERRO] Falha ao abrir {port}: {e}")
        sys.exit(1)

    try:
        if args.mode in ("all", "idle"):
            ok = run_sweep_round(ser, data, "idle", args.min_temp, args.max_temp, args.samples, args.output, port)
            if not ok:
                print("[*] Coleta encerrada pelo usuário.")
                print_summary_table(data, args.min_temp, args.max_temp)
                return

        if args.mode in ("all", "flame"):
            ok = run_sweep_round(ser, data, "flame", args.min_temp, args.max_temp, args.samples, args.output, port)
            if not ok:
                print("[*] Coleta encerrada pelo usuário.")
                print_summary_table(data, args.min_temp, args.max_temp)
                return

        if args.mode in ("all", "off"):
            run_power_off_round(ser, data, args.min_temp, args.max_temp, args.samples, args.output, port)
    except KeyboardInterrupt:
        print("\n[*] Interrompido via Ctrl+C.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    print_summary_table(data, args.min_temp, args.max_temp)
    print(f"\n[+] Dados salvos em: {args.output}")


if __name__ == "__main__":
    main()
