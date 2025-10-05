

# Copy Palanteer binaries from builder stage
COPY --from=@(f"{builder_stage}") @(f"{builder_output_dir}/bin/") /usr/local/bin/

# Copy Palanteer Python wheels from builder stage
COPY --from=@(f"{builder_stage}") @(f"{builder_output_dir}/dist/") /tmp/palanteer_wheels/

# Install Palanteer Python wheel if present (POSIX-compliant)
RUN set -eux; \
	if ls /tmp/palanteer_wheels/*.whl 1> /dev/null 2>&1; then \
		pip3 install /tmp/palanteer_wheels/*.whl; \
	fi

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
