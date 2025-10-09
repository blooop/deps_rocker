# Install Pixi into the user home from pre-populated bundle
RUN mkdir -p ~/.pixi && cp -a /opt/deps_rocker/pixi/. ~/.pixi/
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
RUN echo 'eval "$(pixi shell-hook)"' >> ~/.bashrc
