#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

PYTHON="${SMOLVLA_PYTHON:-}"
WINDOWS_PYTHON=0
if [[ -z "${PYTHON}" ]]; then
  if [[ -x "${WORKSPACE}/openpi/.venv/bin/python" ]]; then
    PYTHON="${WORKSPACE}/openpi/.venv/bin/python"
  elif [[ -x "/mnt/d/Program Files/Python313/python.exe" ]]; then
    # LeRobot 0.6.1 requires Python >= 3.12; use the configured Windows runtime from WSL.
    PYTHON="/mnt/d/Program Files/Python313/python.exe"
    WINDOWS_PYTHON=1
  elif command -v python3 >/dev/null 2>&1; then
    PYTHON="$(command -v python3)"
  else
    PYTHON="$(command -v python)"
  fi
fi

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
if [[ "${WINDOWS_PYTHON}" -eq 1 ]]; then
  export MUJOCO_GL="${MUJOCO_GL:-glfw}"
  unset PYOPENGL_PLATFORM
else
  export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
  export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-osmesa}"
fi
export HF_HOME="${HF_HOME:-${WORKSPACE}/.hf_cache}"

args=(--model-dir "${MODEL_DIR}")
if [[ -d "${VLM_DIR}" ]]; then
  args+=(--vlm-model-dir "${VLM_DIR}")
fi

PYTHON_SCRIPT="${SCRIPT_DIR}/smolvla_infer.py"
if [[ "${WINDOWS_PYTHON}" -eq 1 ]]; then
  args=(--model-dir "$(wslpath -w "${MODEL_DIR}")")
  if [[ -d "${VLM_DIR}" ]]; then
    args+=(--vlm-model-dir "$(wslpath -w "${VLM_DIR}")")
  fi
  PYTHON_SCRIPT="$(wslpath -w "${PYTHON_SCRIPT}")"
fi

exec "${PYTHON}" "${PYTHON_SCRIPT}" "${args[@]}" "$@"
