# syntax=docker/dockerfile:1.4

# Copy Node.js and npm from builder stage to /opt/nodejs in system stage
@(f"COPY --from={builder_stage} {builder_output_dir}/nodejs /opt/nodejs")

# Set proper permissions recursively so all users can read/execute
RUN chmod -R a+rX /opt/nodejs

# Create symlinks in /usr/local/bin for global access (BEFORE user stage needs them)
RUN ln -sf /opt/nodejs/bin/node /usr/local/bin/node && \
    ln -sf /opt/nodejs/bin/npm /usr/local/bin/npm && \
    ln -sf /opt/nodejs/bin/npx /usr/local/bin/npx && \
    ln -sf /opt/nodejs/bin/corepack /usr/local/bin/corepack

# Set SSL config path globally
ENV OPENSSL_CONF=/opt/nodejs/ssl/openssl.cnf

# Disable npm update notifier and fund messages
ENV NPM_CONFIG_UPDATE_NOTIFIER=false
ENV NPM_CONFIG_FUND=false
