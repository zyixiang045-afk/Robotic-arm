#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

PYTHON="${SMOLVLA_PYTHON:-${WORKSPACE}/openpi/.venv/bin/python}"
MODEL_DIR="${SMOLVLA_MODEL_DIR:-${WORKSPACE}/models/smolvla_base}"
VLM_DIR="${SMOLVLA_VLM_MODEL_DIR:-${WORKSPACE}/models/SmolVLM2-500M-Video-Instruct}"
OSMESA_LIB="${SMOLVLA_OSMESA_LIB:-/lib/x86_64-linux-gnu/libOSMesa.so}"

if [[ ! -x "${PYTHON}" ]]; then
  echo "Missing Python interpreter: ${PYTHON}" >&2
  exit 1
fi

if [[ -f "${OSMESA_LIB}" ]]; then
  export LD_PRELOAD="${OSMESA_LIB}${SMOLVLA_EXTRA_LD_PRELOAD:+:${SMOLVLA_EXTRA_LD_PRELOAD}}"
fi
export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-osmesa}"
export HF_HOME="${HF_HOME:-${WORKSPACE}/.hf_cache}"

args=(--model-dir "${MODEL_DIR}")
if [[ -d "${VLM_DIR}" ]]; then
  args+=(--vlm-model-dir "${VLM_DIR}")
fi

exec "${PYTHON}" "${SCRIPT_DIR}/smolvla_infer.py" "${args[@]}" "$@"
