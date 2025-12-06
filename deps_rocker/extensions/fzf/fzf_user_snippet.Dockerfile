# Install fzf via pixi
RUN pixi global install fzf

# Set up fzf shell integration
# Find fzf package files in pixi environment
RUN bash -c ' \
    FZF_BASE=$(find ~/.pixi/envs -name "fzf" -type d 2>/dev/null | head -1) && \
    if [ -n "$FZF_BASE" ] && [ -d "$FZF_BASE/shell" ]; then \
        echo "# fzf setup" >> ~/.bashrc && \
        echo "[ -f $FZF_BASE/shell/completion.bash ] && source $FZF_BASE/shell/completion.bash" >> ~/.bashrc && \
        echo "[ -f $FZF_BASE/shell/key-bindings.bash ] && source $FZF_BASE/shell/key-bindings.bash" >> ~/.bashrc; \
    fi && \
    echo "cdfzf() { file=\"\$(fzf)\"; [ -n \"\$file\" ] && cd \"\$(dirname \"\$file\")\"; }" >> ~/.bashrc'
