
# This is a Dockerfile snippet, not a full Dockerfile. It is intended to be injected into a larger Dockerfile and does not require a FROM statement.

# Ensure docker group exists and add user to docker group if not already present
RUN groupadd -f docker && \
    username="${USER:-$(getent passwd 1000 | cut -d: -f1)}"; \
    if [ -n "$username" ] && id "$username" >/dev/null 2>&1; then \
        if ! id -nG "$username" | grep -qw docker; then \
            usermod -aG docker "$username"; \
            echo "Added $username to docker group."; \
        else \
            echo "$username already in docker group."; \
        fi; \
        id "$username"; \
    else \
        echo "Warning: USER not set and no user with UID 1000 found, skipping docker group assignment"; \
    fi && \
    # Fix docker.sock permissions if present
    if [ -S /var/run/docker.sock ]; then \
        chown root:docker /var/run/docker.sock || true; \
        chmod 660 /var/run/docker.sock || true; \
        echo "Set permissions on /var/run/docker.sock"; \
    else \
        echo "/var/run/docker.sock not present at build time, will be handled at runtime."; \
    fi
