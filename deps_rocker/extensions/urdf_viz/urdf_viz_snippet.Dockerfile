# Install urdf-viz binary from builder stage
COPY --from=@builder_stage@ @builder_output_dir@/urdf-viz /usr/local/bin/urdf-viz
RUN chmod +x /usr/local/bin/urdf-viz
