# syntax=docker/dockerfile:1.4
ARG NEOVIM_VERSION=v0.11.4

# Copy neovim binary built in the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/nvim /opt/nvim")
RUN ln -sf /opt/nvim/bin/nvim /usr/local/bin/nvim
