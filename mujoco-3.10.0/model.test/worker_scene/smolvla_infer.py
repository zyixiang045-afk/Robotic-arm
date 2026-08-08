#!/usr/bin/env python3
"""Compatibility wrapper for the historical SmolVLA inference entrypoint."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[2]
SOURCE_REF = os.environ.get("SMOLVLA_SOURCE_REF", "HEAD")
SOURCE_PATH = "mujoco-3.10.0/model/worker_scene/smolvla_infer.py"


def main() -> None:
    try:
        source = subprocess.check_output(
            ["git", "show", f"{SOURCE_REF}:{SOURCE_PATH}"],
            cwd=str(WORKSPACE),
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        raise SystemExit(
            "Unable to load the historical smolvla_infer.py from git history."
        ) from exc

    namespace = {"__file__": str(HERE / "smolvla_infer.py"), "__name__": "__main__"}
    exec(compile(source, namespace["__file__"], "exec"), namespace)


if __name__ == "__main__":
    main()
