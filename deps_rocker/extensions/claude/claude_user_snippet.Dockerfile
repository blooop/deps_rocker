# Install Claude Code CLI for the user using cached binary (pinned to specific version for reproducibility)
@(f"COPY --from={builder_stage} {builder_output_dir}/claude-binary /tmp/claude-binary")
RUN mkdir -p "$HOME/.local/bin" && \
    cp /tmp/claude-binary "$HOME/.local/bin/claude" && \
    chmod +x "$HOME/.local/bin/claude" && \
    echo "Claude CLI installed from cached binary to $HOME/.local/bin/claude"
