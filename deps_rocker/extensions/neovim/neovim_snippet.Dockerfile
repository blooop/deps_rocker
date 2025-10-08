# syntax=docker/dockerfile:1.4
@(f"ARG NEOVIM_VERSION={NEOVIM_VERSION}")

# Copy neovim binary built in the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/nvim /opt/nvim")
RUN ln -sf /opt/nvim/bin/nvim /usr/local/bin/nvim
