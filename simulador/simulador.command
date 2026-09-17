#!/bin/bash
# Launcher do simulador (Mac: duplo clique em simulador.command; Linux: ./simulador.sh).
# 1ª vez: cria o venv e instala as deps. Depois: sobe o adapter, que abre o browser.
cd "$(dirname "$0")" || exit 1
if [ ! -x .venv/bin/python ]; then
  echo "== primeira vez: criando ambiente =="
  python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt || { echo "falhou a instalação"; read -r; exit 1; }
fi
exec .venv/bin/python can_adapter.py
