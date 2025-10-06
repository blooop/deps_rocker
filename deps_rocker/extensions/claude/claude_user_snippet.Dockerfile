# Install Claude Code CLI for the user
@(f"COPY --from={builder_stage} {builder_output_dir}/install.sh /tmp/claude-install.sh")
RUN bash /tmp/claude-install.sh
