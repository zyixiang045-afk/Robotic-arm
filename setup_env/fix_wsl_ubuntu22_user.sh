#!/usr/bin/env bash
set -euo pipefail

if ! id daniel >/dev/null 2>&1; then
  useradd -m -s /bin/bash daniel
fi
usermod -aG sudo daniel || true

cat >/etc/wsl.conf <<'EOF'
[user]
default=daniel
EOF

mkdir -p /home/daniel/ros2_ws/src

if ! grep -qxF "source /opt/ros/humble/setup.bash" /home/daniel/.bashrc 2>/dev/null; then
  printf '\nsource /opt/ros/humble/setup.bash\n' >> /home/daniel/.bashrc
fi

chown -R daniel:daniel /home/daniel/ros2_ws || true
chown daniel:daniel /home/daniel/.bashrc || true

echo "Ubuntu 22.04 user configuration updated."
