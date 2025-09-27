# Install nvm, node and npm
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=24.9.0

# Create nvm directory
RUN mkdir -p $NVM_DIR

# Download and install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.0/install.sh | bash

# Install node and npm using nvm
RUN bash -c "source $NVM_DIR/nvm.sh && nvm install $NODE_VERSION && nvm use $NODE_VERSION && nvm alias default $NODE_VERSION"

# Add node and npm to path
ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
