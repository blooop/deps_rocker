# Add current user to docker group for non-root Docker access
RUN groupadd -f docker && \
    if [ -n "$USER" ] && id "$USER" >/dev/null 2>&1; then \
        usermod -aG docker "$USER"; \
    else \
        echo "Warning: USER not set or user does not exist, skipping docker group assignment"; \
    fi
