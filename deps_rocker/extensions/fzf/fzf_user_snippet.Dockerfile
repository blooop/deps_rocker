# Install fzf from staged source (binary already present from builder stage)
RUN rm -rf ~/.fzf && mkdir -p ~/.fzf && cp -a /opt/deps_rocker/fzf/. ~/.fzf/

# Set up completion and key-bindings by sourcing shell files directly (no install script)
RUN echo '# fzf setup' >> ~/.bashrc && \
    echo 'export PATH="$HOME/.fzf/bin:$PATH"' >> ~/.bashrc && \
    echo '[ -f ~/.fzf/shell/completion.bash ] && source ~/.fzf/shell/completion.bash' >> ~/.bashrc && \
    echo '[ -f ~/.fzf/shell/key-bindings.bash ] && source ~/.fzf/shell/key-bindings.bash' >> ~/.bashrc && \
    echo 'cdfzf() { file="$(fzf)"; [ -n "$file" ] && cd "$(dirname "$file")"; }' >> ~/.bashrc
