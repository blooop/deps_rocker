# Install Rust toolchain from cached builder stage
COPY --from=@builder_stage@ @builder_output_dir@/root/.cargo /root/.cargo
COPY --from=@builder_stage@ @builder_output_dir@/root/.rustup /root/.rustup
COPY --from=@builder_stage@ @builder_output_dir@/cargo-env.sh /etc/profile.d/cargo-env.sh
RUN chmod 644 /etc/profile.d/cargo-env.sh && \
    echo 'source /etc/profile.d/cargo-env.sh' >> /etc/bash.bashrc && \
    echo 'source /etc/profile.d/cargo-env.sh' >> /root/.bashrc
ENV PATH="/root/.cargo/bin:${PATH}"
