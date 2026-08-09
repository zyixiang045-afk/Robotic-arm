#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [[ -n "${SMOLVLA_PYTHON:-}" ]]; then
  PYTHON="${SMOLVLA_PYTHON}"
elif [[ -x "${WORKSPACE}/openpi/.venv/bin/python" ]]; then
  PYTHON="${WORKSPACE}/openpi/.venv/bin/python"
else
  PYTHON="python3"
fi

OSMESA_LIB="${SMOLVLA_OSMESA_LIB:-/lib/x86_64-linux-gnu/libOSMesa.so}"
if [[ -f "${OSMESA_LIB}" ]]; then
  export LD_PRELOAD="${OSMESA_LIB}${SMOLVLA_EXTRA_LD_PRELOAD:+:${SMOLVLA_EXTRA_LD_PRELOAD}}"
fi

export MUJOCO_GL="${MUJOCO_GL:-osmesa}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-osmesa}"

exec "${PYTHON}" "${SCRIPT_DIR}/replay_smolvla_trace.py" "$@"
