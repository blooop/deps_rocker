# Install nvm, node and npm
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=24.9.0
ENV NPM_VERSION=11.6.1

# Create nvm directory
RUN mkdir -p $NVM_DIR

# Copy nvm install script from builder output
@(f"COPY --from={builder_stage} {builder_output_dir}/install.sh /tmp/install.sh")
RUN bash /tmp/install.sh

# Install node and npm using nvm, then upgrade npm to specific version
RUN bash -c "source $NVM_DIR/nvm.sh && nvm install $NODE_VERSION && nvm use $NODE_VERSION && nvm alias default $NODE_VERSION && npm install -g npm@@$NPM_VERSION"

# Verify installed npm version
RUN bash -c "source $NVM_DIR/nvm.sh && echo 'Installed npm version:' && npm --version"

# Add node and npm to path
ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
