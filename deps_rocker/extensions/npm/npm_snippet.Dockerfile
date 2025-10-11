
ENV NODE_VERSION=24.9.0
# Install nvm, node and npm
ENV NVM_DIR=/usr/local/nvm

ENV NPM_CONFIG_UPDATE_NOTIFIER=false 
ENV NPM_CONFIG_FUND=false 

# Copy pre-installed nvm directory from builder
@(f"COPY --from={builder_stage} $NVM_DIR $NVM_DIR")

# Add node and npm to path
ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
