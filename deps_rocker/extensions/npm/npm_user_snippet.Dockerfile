# Node.js and npm are synced at runtime via pixi-global manifest

# Add npm global bin directory to PATH for login shells
RUN echo 'export PATH="$HOME/.pixi/envs/nodejs/bin:$PATH"' >> ~/.profile

# Create symlinks in /usr/local/bin for non-interactive shell access
RUN sudo ln -sf ~/.pixi/envs/nodejs/bin/node /usr/local/bin/node && \
    sudo ln -sf ~/.pixi/envs/nodejs/bin/npm /usr/local/bin/npm && \
    sudo ln -sf ~/.pixi/envs/nodejs/bin/npx /usr/local/bin/npx

# Configure shell to use bash and source profile for subsequent RUN commands
SHELL ["/bin/bash", "-l", "-c"]

# Disable npm update notifier and fund messages
ENV NPM_CONFIG_UPDATE_NOTIFIER=false
ENV NPM_CONFIG_FUND=false
