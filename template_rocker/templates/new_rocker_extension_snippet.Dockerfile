RUN curl -fsSL https://pixi.sh/install.sh | bash
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc