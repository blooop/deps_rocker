# Copy Palanteer binaries from builder stage
COPY --from=@builder_stage@ @builder_output_dir@/bin/ /usr/local/bin/

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
