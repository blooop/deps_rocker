# syntax=docker/dockerfile:1.4
ARG FZF_VERSION=@fzf_version@

# Make fzf source available for user install step
@(f"COPY --from={builder_stage} {builder_output_dir}/fzf /opt/deps_rocker/fzf")
RUN chmod -R a+rX /opt/deps_rocker/fzf
