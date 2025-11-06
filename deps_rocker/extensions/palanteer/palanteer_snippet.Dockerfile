# syntax=docker/dockerfile:1.4

# Copy palanteer binaries from builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/* /usr/local/bin/")

# Ensure binaries are executable
RUN chmod +x /usr/local/bin/palanteer* || true

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
