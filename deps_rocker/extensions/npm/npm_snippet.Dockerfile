# Install nvm, node and npm
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=24.9.0
ENV NPM_VERSION=11.6.1

# Create nvm directory
RUN mkdir -p $NVM_DIR


# Download and install nvm using BuildKit cache mount for the install script
RUN --mount=type=cache,target=/tmp/nvm-install-cache \
	curl -o /tmp/nvm-install-cache/install.sh https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh && \
	bash /tmp/nvm-install-cache/install.sh

# Install node and npm using nvm, then upgrade npm to specific version
RUN bash -c "source $NVM_DIR/nvm.sh && nvm install $NODE_VERSION && nvm use $NODE_VERSION && nvm alias default $NODE_VERSION && npm install -g npm@@$NPM_VERSION"

# Verify installed npm version
RUN bash -c "source $NVM_DIR/nvm.sh && echo 'Installed npm version:' && npm --version"

# Add node and npm to path
ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
