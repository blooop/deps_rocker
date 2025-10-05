# syntax=docker/dockerfile:1.4
ARG NODE_VERSION=@node_version@
ARG NPM_VERSION=@npm_version@
ARG NVM_VERSION=@nvm_version@

# Install nvm, node, and npm from builder stage artifacts
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=24.9.0
ENV NPM_VERSION=11.6.1

@(f"COPY --from={builder_stage} {builder_output_path}node /usr/local/bin/")
@(f"COPY --from={builder_stage} {builder_output_path}npm /usr/local/bin/")
@(f"COPY --from={builder_stage} {builder_output_path}npx /usr/local/bin/")
@(f"COPY --from={builder_stage} {builder_output_path}node-env.sh /etc/profile.d/node-env.sh")

RUN chmod +x /usr/local/bin/node /usr/local/bin/npm /usr/local/bin/npx && \
    chmod 644 /etc/profile.d/node-env.sh && \
    echo '. /etc/profile.d/node-env.sh' >> /etc/bash.bashrc && \
    echo '. /etc/profile.d/node-env.sh' >> /root/.bashrc

RUN node --version && npm --version
