#!/usr/bin/env bash
# Atalho para iniciar o Painel Virtual do Aquecedor Rinnai em Tempo Real

PYTHON_BIN=""
if [ -f "$HOME/.platformio/penv/bin/python3" ]; then
    PYTHON_BIN="$HOME/.platformio/penv/bin/python3"
elif command -v python3 &>/dev/null; then
    PYTHON_BIN="python3"
else
    echo "Erro: Python 3 não encontrado."
    exit 1
fi

exec "$PYTHON_BIN" "$(dirname "$0")/tools/live_dashboard.py" "$@"
