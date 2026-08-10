#!/usr/bin/env bash
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
TARGET_HOME="${TARGET_HOME:-/home/daniel}"
MUJOCO_HOME="${MUJOCO_HOME:-$TARGET_HOME/mujoco-3.10.0}"

apt-get update -y
apt-get install -y curl tar pkg-config python3-pip

if [ ! -d "$MUJOCO_HOME" ]; then
  curl -L --fail --retry 3 \
    -o "$TARGET_HOME/mujoco-3.10.0-linux-x86_64.tar.gz" \
    https://github.com/google-deepmind/mujoco/releases/download/3.10.0/mujoco-3.10.0-linux-x86_64.tar.gz
  tar -xzf "$TARGET_HOME/mujoco-3.10.0-linux-x86_64.tar.gz" -C "$TARGET_HOME"
fi

grep -qxF "export MUJOCO_HOME=\"$MUJOCO_HOME\"" "$TARGET_HOME/.bashrc" || \
  printf '\nexport MUJOCO_HOME="%s"\nexport PATH="$MUJOCO_HOME/bin:$PATH"\nexport CPATH="$MUJOCO_HOME/include${CPATH:+:$CPATH}"\nexport LIBRARY_PATH="$MUJOCO_HOME/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"\nexport LD_LIBRARY_PATH="$MUJOCO_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"\nexport PKG_CONFIG_PATH="$HOME/.local/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"\n' "$MUJOCO_HOME" >> "$TARGET_HOME/.bashrc"

mkdir -p "$TARGET_HOME/.local/lib/pkgconfig"
cat >"$TARGET_HOME/.local/lib/pkgconfig/mujoco.pc" <<EOF
prefix=$MUJOCO_HOME
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: mujoco
Description: MuJoCo physics engine
Version: 3.10.0
Cflags: -I\${includedir}
Libs: -L\${libdir} -Wl,-rpath,\${libdir} -lmujoco -ldl -pthread
EOF

python3 -m pip install --user --upgrade 'mujoco==3.2.3' 'zipp>=3.1.0'

chown -R daniel:daniel "$TARGET_HOME/.local" "$MUJOCO_HOME" || true

echo "MuJoCo environment is ready."
