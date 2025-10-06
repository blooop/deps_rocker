# Install Claude Code CLI for the user (pinned to specific version for reproducibility)
@(f"COPY --from={builder_stage} {builder_output_dir}/install.sh /tmp/claude-install.sh")
RUN bash /tmp/claude-install.sh 2.0.8
