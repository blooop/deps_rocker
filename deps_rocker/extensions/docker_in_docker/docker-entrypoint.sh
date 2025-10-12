
#!/bin/sh
set -eu

# Ensure we run as root here
if [[ "$(id -u)" -ne 0 ]]; then
  echo "docker-entrypoint.sh must run as root"; exit 1
fi

# Determine the interactive user we should drop to.
TARGET_USER="${TARGET_USER:-${USER:-}}"
if [[ -z "${TARGET_USER}" || "${TARGET_USER}" == "root" ]]; then
  # best-effort: pick uid 1000 account if available
  TARGET_USER="$(getent passwd 1000 | cut -d: -f1 || true)"
  [[ -z "${TARGET_USER}" ]] && TARGET_USER="root"
fi

# Ensure docker group and add user
#!/bin/sh
set -eu
echo "[docker-entrypoint.sh] Starting DinD entrypoint..."

# Ensure we run as root here
if [ "$(id -u)" -ne 0 ]; then
  echo "docker-entrypoint.sh must run as root"; exit 1
fi

# Determine the interactive user we should drop to.
TARGET_USER="${TARGET_USER:-${USER:-}}"
if [ -z "$TARGET_USER" ] || [ "$TARGET_USER" = "root" ]; then
  TARGET_USER="$(getent passwd 1000 | cut -d: -f1 2>/dev/null)"
  if [ -z "$TARGET_USER" ]; then
    TARGET_USER="root"
  fi
fi

# Ensure docker group and add user
groupadd -f docker
if id -u "$TARGET_USER" >/dev/null 2>&1; then
  usermod -aG docker "$TARGET_USER" || true
fi

# Start dockerd if not running
if ! pgrep dockerd >/dev/null 2>&1; then
  /usr/bin/dockerd --host=unix:///var/run/docker.sock \
                   --exec-root=/var/run/docker \
                   --data-root=/var/lib/docker &
fi

# wait for the socket (loop up to 60 times)
count=0
while [ "$count" -lt 60 ]; do
  if [ -S /var/run/docker.sock ]; then
    break
  fi
  sleep 0.5
  count=$((count + 1))
done
if [ ! -S /var/run/docker.sock ]; then
  echo "dockerd failed to create /var/run/docker.sock"; ps aux | grep dockerd || true; exit 1
fi

# fix permissions on the socket for docker group
chgrp docker /var/run/docker.sock || true
chmod 660 /var/run/docker.sock || true

# drop privileges to the intended user and exec the default CMD (interactive shell via rocker)
if [ "$TARGET_USER" != "root" ]; then
  exec gosu "$TARGET_USER" "$@"

set -eu

echo "[docker-entrypoint] UID: $(id -u), GID: $(id -g), USER: $(id -un)"
echo "[docker-entrypoint] Groups: $(id -Gn)"
echo "[docker-entrypoint] Docker socket: $(ls -l /var/run/docker.sock 2>&1 || echo 'not found')"

# Ensure user is in docker group if group exists
if getent group docker >/dev/null; then
  if ! id -nG "$USER" | grep -qw docker; then
    echo "[docker-entrypoint] Adding $USER to docker group"
    sudo usermod -aG docker "$USER"
  fi
fi

# Fix permissions on docker.sock if needed
if [ -S /var/run/docker.sock ]; then
  sudo chown root:docker /var/run/docker.sock || true
  sudo chmod 660 /var/run/docker.sock || true
fi

# Start Docker daemon if not running
if ! pgrep dockerd >/dev/null; then
  echo "[docker-entrypoint] Starting Docker daemon..."
  sudo dockerd > /tmp/dockerd.log 2>&1 &
  sleep 2
  if ! pgrep dockerd >/dev/null; then
    echo "[docker-entrypoint] ERROR: Docker daemon failed to start. See /tmp/dockerd.log"
    exit 1
  fi
fi

# Print Docker version for diagnostics
docker --version || echo "[docker-entrypoint] Docker not available"

# Interactive mode: drop to shell, else continue
if [ "$1" = "-i" ] || [ -t 0 ]; then
  echo "[docker-entrypoint] Interactive shell detected. Dropping to shell."
  exec "$SHELL"
else
  echo "[docker-entrypoint] Non-interactive mode."
  exec "$@"
fi
