# Configure npm to use user-writable prefix for global installs
RUN mkdir -p ~/.npm-global && \
    npm config set prefix ~/.npm-global

# Add npm global bin to PATH for user-installed packages
RUN echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> ~/.bashrc && \
    echo 'export PATH="$HOME/.npm-global/bin:$PATH"' >> ~/.profile
