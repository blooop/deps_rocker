
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
else
  exec "$@"
fi
