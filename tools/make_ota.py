#!/usr/bin/env python3
"""
Gera a imagem OTA Zigbee (.ota) a partir do firmware.bin do PlatformIO e o índice
para o Zigbee2MQTT (zigbee_ota_override_index_location).

Uso:
  python3 tools/make_ota.py --version 0x00010002 [--bin firmware_nanoc6/.pio/build/m5nanoc6/firmware.bin]
O valor de --version deve ser IGUAL ao RINNAI_FW_VERSION compilado no firmware.
Saída: z2m/ota/rinnai_nanoc6_<versao>.ota e z2m/ota/ota_index.json
"""
import argparse, json, hashlib, os, struct

MANUFACTURER = 0x1001
IMAGE_TYPE = 0x1011
HEADER_VERSION = 0x0100
HEADER_LEN = 56
FIELD_CTRL = 0x0000
STACK_VERSION = 0x0002  # ZigBee Pro
HEADER_STRING = b"RinnaiNanoC6 OTA"
TAG_UPGRADE_IMAGE = 0x0000


def build(fw: bytes, version: int) -> bytes:
    sub = struct.pack("<HI", TAG_UPGRADE_IMAGE, len(fw)) + fw
    total = HEADER_LEN + len(sub)
    hs = HEADER_STRING.ljust(32, b"\x00")[:32]
    header = struct.pack("<IHHHHHIH32sI", 0x0BEEF11E, HEADER_VERSION, HEADER_LEN,
                         FIELD_CTRL, MANUFACTURER, IMAGE_TYPE, version, STACK_VERSION, hs, total)
    assert len(header) == HEADER_LEN, len(header)
    return header + sub


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True, help="ex.: 0x00010002")
    ap.add_argument("--bin", default=os.path.join(root, "firmware_nanoc6/.pio/build/m5nanoc6/firmware.bin"))
    a = ap.parse_args()
    ver = int(a.version, 0)
    fw = open(a.bin, "rb").read()
    img = build(fw, ver)
    out_dir = os.path.join(root, "z2m", "ota")
    os.makedirs(out_dir, exist_ok=True)
    name = f"rinnai_nanoc6_{ver:08x}.ota"
    open(os.path.join(out_dir, name), "wb").write(img)
    entry = {
        "fileName": name,
        "fileVersion": ver,
        "fileSize": len(img),
        "manufacturerCode": MANUFACTURER,
        "imageType": IMAGE_TYPE,
        "sha512": hashlib.sha512(img).hexdigest(),
        "url": name,
    }
    idx_path = os.path.join(out_dir, "ota_index.json")
    idx = json.load(open(idx_path)) if os.path.exists(idx_path) else []
    idx = [e for e in idx if e["fileVersion"] != ver] + [entry]
    json.dump(idx, open(idx_path, "w"), indent=2)
    print(f"OK: {name} ({len(img)} bytes) + ota_index.json")


if __name__ == "__main__":
    main()
