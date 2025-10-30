# syntax=docker/dockerfile:1.4
ARG FOXGLOVE_VERSION=@FOXGLOVE_VERSION@

@(f"COPY --from={builder_stage} {builder_output_dir}/foxglove-studio.deb /tmp/foxglove-studio.deb")
@(f"COPY --from={builder_stage} {builder_output_dir}/version.txt /tmp/foxglove-version.txt")

RUN bash -c "set -euxo pipefail && \
    printf '%s' \"${FOXGLOVE_VERSION}\" > /tmp/expected-foxglove-version && \
    cmp -s /tmp/expected-foxglove-version /tmp/foxglove-version.txt && \
    dpkg-deb --info /tmp/foxglove-studio.deb > /dev/null 2>&1 && \
    dpkg -i /tmp/foxglove-studio.deb && \
    rm /tmp/foxglove-studio.deb /tmp/foxglove-version.txt /tmp/expected-foxglove-version && \
    rm -f /etc/apt/sources.list.d/foxglove-studio.list"
