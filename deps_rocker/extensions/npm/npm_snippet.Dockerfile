# Install Node.js and npm via pixi global install
RUN pixi global install nodejs

# Disable npm update notifier and fund messages
ENV NPM_CONFIG_UPDATE_NOTIFIER=false
ENV NPM_CONFIG_FUND=false
