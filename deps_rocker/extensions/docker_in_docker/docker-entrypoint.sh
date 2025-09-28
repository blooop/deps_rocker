#!/bin/bash
set -e

# Initialize Docker if needed
if [ -f "/usr/local/share/docker-init.sh" ]; then
    /usr/local/share/docker-init.sh
fi

# Execute the original command
exec "$@"