# syntax=docker/dockerfile:1.4

# Copy Rust toolchain from builder stage to /opt/rust in system stage
@(f"COPY --from={builder_stage} {builder_output_dir}/rust /opt/rust")

# Set proper permissions recursively so all users can read/execute
RUN chmod -R a+rX /opt/rust

# Create symlinks in /usr/local/bin for global access (BEFORE user stage needs them)
RUN ln -sf /opt/rust/bin/cargo /usr/local/bin/cargo && \
    ln -sf /opt/rust/bin/rustc /usr/local/bin/rustc && \
    ln -sf /opt/rust/bin/rustdoc /usr/local/bin/rustdoc && \
    ln -sf /opt/rust/bin/rust-analyzer /usr/local/bin/rust-analyzer 2>/dev/null || true && \
    ln -sf /opt/rust/bin/rustfmt /usr/local/bin/rustfmt 2>/dev/null || true

# Set environment variables for cargo home
ENV CARGO_HOME=/opt/rust
ENV PATH="/opt/rust/bin:${PATH}"
