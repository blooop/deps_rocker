# syntax=docker/dockerfile:1.4
ARG LAZYGIT_VERSION=@LAZYGIT_VERSION@

# Copy lazygit binary built in the builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/lazygit /usr/local/bin/lazygit")

# Add an alias for lazygit to .bashrc
RUN echo "alias lg='lazygit'" >> ~/.bashrc
