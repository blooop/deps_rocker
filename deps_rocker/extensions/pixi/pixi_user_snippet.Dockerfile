RUN curl -fsSL https://pixi.sh/install.sh | bash
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
