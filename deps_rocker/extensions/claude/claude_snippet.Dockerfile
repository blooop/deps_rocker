# Install global wrapper to ensure PATH includes ~/.local/bin
@(f"COPY --from={builder_stage} {builder_output_dir}/claude-wrapper.sh /usr/local/bin/claude")
RUN chmod +x /usr/local/bin/claude && \
    echo 'Claude wrapper installed. Will install claude to user home in user section.'
