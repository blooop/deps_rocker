RUN --mount=type=cache,target=/root/.cache/lazygit-cache \
	mkdir -p /root/.cache/lazygit-cache && \
	LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" \
	    | grep -Eo '"tag_name": "v[^" ]*"' \
	    | head -n1 \
	    | sed 's/.*"v\([^" ]*\)".*/\1/') && \
	echo "Lazygit version: ${LAZYGIT_VERSION}" && \
	tarball="/root/.cache/lazygit-cache/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" && \
	if [ ! -f "${tarball}" ]; then \
		curl -sSL "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" -o "${tarball}"; \
	fi && \
	ls -lh "${tarball}" && \
	tar -xzf "${tarball}" lazygit && \
	install lazygit /usr/local/bin && \
	rm lazygit
