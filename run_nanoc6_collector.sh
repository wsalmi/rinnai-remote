#!/bin/bash
# Executa o coletor interativo do M5NanoC6 utilizando o ambiente Python do PlatformIO
PYTHON_BIN="$HOME/.platformio/penv/bin/python3"

if [ ! -f "$PYTHON_BIN" ]; then
    PYTHON_BIN="python3"
fi

exec "$PYTHON_BIN" "$(dirname "$0")/tools/nanoc6_collector.py" "$@"
