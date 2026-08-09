#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

PYTHON="${SMOLVLA_PYTHON:-${WORKSPACE}/openpi/.venv/bin/python}"
DEST="${1:-${WORKSPACE}/models/SmolVLM2-500M-Video-Instruct}"

if [[ ! -x "${PYTHON}" ]]; then
  echo "Missing Python interpreter: ${PYTHON}" >&2
  exit 1
fi

mkdir -p "$(dirname "${DEST}")"
export HF_HOME="${HF_HOME:-${WORKSPACE}/.hf_cache}"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

exec "${PYTHON}" -m huggingface_hub.commands.huggingface_cli download \
  HuggingFaceTB/SmolVLM2-500M-Video-Instruct \
  --local-dir "${DEST}" \
  --include "*.json" "*.txt" "*.safetensors" "*.model" \
  --exclude "onnx/*" "*.onnx" \
  --max-workers "${HF_MAX_WORKERS:-4}"
