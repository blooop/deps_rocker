# syntax=docker/dockerfile:1.4

# Install Rust toolchain from cached builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/root/.cargo /root/.cargo")
@(f"COPY --from={builder_stage} {builder_output_dir}/root/.rustup /root/.rustup")
@(f"COPY --from={builder_stage} {builder_output_dir}/cargo-env.sh /etc/profile.d/cargo-env.sh")
RUN chmod 644 /etc/profile.d/cargo-env.sh && \
    echo 'source /etc/profile.d/cargo-env.sh' >> /etc/bash.bashrc && \
    echo 'source /etc/profile.d/cargo-env.sh' >> /root/.bashrc
ENV PATH="/root/.cargo/bin:${PATH}"
