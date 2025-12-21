# Install Node.js and npm via pixi global install
RUN pixi global install nodejs

# Add npm global bin directory to PATH for all shells
ENV PATH="$HOME/.pixi/envs/nodejs/bin:$PATH"

# Disable npm update notifier and fund messages
ENV NPM_CONFIG_UPDATE_NOTIFIER=false
ENV NPM_CONFIG_FUND=false
