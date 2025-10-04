RUN --mount=type=cache,target=/tmp/pixi-install-cache \
	curl -fsSL https://pixi.sh/install.sh -o /tmp/pixi-install-cache/install.sh && \
	bash /tmp/pixi-install-cache/install.sh
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
