# Copy lazygit binary built in the builder stage
COPY --from=@builder_stage@ @builder_output_dir@/lazygit /usr/local/bin/lazygit
