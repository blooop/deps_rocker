ENV NODE_VERSION=24.9.0
# Install nvm, node and npm
ENV NVM_DIR=/usr/local/nvm
ENV NPM_VERSION=11.6.1

# Copy pre-installed nvm directory from builder
@(f"COPY --from={builder_stage} $NVM_DIR $NVM_DIR")

# Upgrade npm to specific version
RUN bash -c "source $NVM_DIR/nvm.sh && nvm use $NODE_VERSION && npm install -g npm@@$NPM_VERSION"

# Verify installed npm version
RUN bash -c "source $NVM_DIR/nvm.sh && echo 'Installed npm version:' && npm --version"

# Add node and npm to path
ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
