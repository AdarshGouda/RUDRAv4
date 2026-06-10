#!/usr/bin/env bash
set -euo pipefail

MODEL="${1:-qwen2.5:3b}"
LOG_DIR="${HOME}/.ros/log"
LOG_FILE="${LOG_DIR}/ollama-rudra.log"

mkdir -p "${LOG_DIR}"

if ! command -v ollama >/dev/null 2>&1; then
  cat <<'MSG'
Ollama is not installed on this NUC.

Install it first:
  curl -fsSL https://ollama.com/install.sh | sh

Then run:
  bash scripts/start_ollama.sh
MSG
  exit 127
fi

if curl -fsS --max-time 2 http://localhost:11434/api/tags >/dev/null 2>&1; then
  echo "Ollama is already running on http://localhost:11434"
else
  echo "Starting Ollama in the background..."
  nohup ollama serve >"${LOG_FILE}" 2>&1 &
  sleep 2
fi

if ! curl -fsS --max-time 5 http://localhost:11434/api/tags >/dev/null; then
  echo "Ollama did not respond. Check ${LOG_FILE}" >&2
  exit 1
fi

if ! ollama list | awk '{print $1}' | grep -Fxq "${MODEL}"; then
  echo "Pulling ${MODEL}. This can take a while..."
  ollama pull "${MODEL}"
fi

echo "Ollama is ready with ${MODEL}"
