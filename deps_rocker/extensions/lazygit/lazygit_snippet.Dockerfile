# syntax=docker/dockerfile:1.4
ARG LAZYGIT_VERSION=@lazygit_version@

# Copy lazygit binary built in the builder stage
COPY --from=@builder_stage@ @builder_output_dir@/lazygit /usr/local/bin/lazygit
