RUN --mount=type=cache,target=/root/.cache/pixi-install-cache \
	sh -c 'if [ "$(id -u)" = 0 ]; then \
		CACHE_DIR="/root/.cache/pixi-install-cache"; \
	else \
		CACHE_DIR="${HOME:-/tmp}/.cache/pixi-install-cache"; \
	fi; \
	mkdir -p "$CACHE_DIR" && \
	SCRIPT_PATH="$CACHE_DIR/install.sh" && \
	if [ ! -f "$SCRIPT_PATH" ]; then \
		curl -fsSL https://pixi.sh/install.sh -o "$SCRIPT_PATH"; \
	fi; \
	bash "$SCRIPT_PATH"'
RUN echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
RUN echo 'eval "$(pixi completion --shell bash)"' >> ~/.bashrc
