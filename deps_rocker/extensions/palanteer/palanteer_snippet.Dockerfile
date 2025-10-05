# Copy Palanteer binaries from builder stage
COPY --from=palanteer_builder /opt/deps_rocker/palanteer/bin/ /usr/local/bin/

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
