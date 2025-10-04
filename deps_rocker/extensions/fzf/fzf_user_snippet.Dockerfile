
# Install fzf from source using BuildKit cache for the git repo (use root-owned cache dir if root, else fallback to $HOME)
RUN --mount=type=cache,target=/root/.cache/fzf-git-cache \
	cache_dir="/root/.cache/fzf-git-cache"; \
	if [ "$(id -u)" != "0" ]; then cache_dir="$HOME/.cache/fzf-git-cache"; fi; \
	mkdir -p "$cache_dir" && \
	if [ ! -d "$cache_dir/fzf" ]; then \
		git clone --depth 1 https://github.com/junegunn/fzf.git "$cache_dir/fzf"; \
	fi && \
	cp -r "$cache_dir/fzf" ~/.fzf && \
	~/.fzf/install --all

# Add cdfzf function to bashrc
RUN echo 'cdfzf() { file="$(fzf)"; [ -n "$file" ] && cd "$(dirname "$file")"; }' >> ~/.bashrc
