# syntax=docker/dockerfile:1.4
@(f"FROM {base_image} AS {builder_stage}")

# Clone and build palanteer using BuildKit cache for both git repo and build artifacts
RUN --mount=type=cache,target=/root/.cache/palanteer-git-cache,id=palanteer-git-cache \
    --mount=type=cache,target=/root/.cache/palanteer-build-cache,id=palanteer-build-cache \
    bash -c 'set -euxo pipefail && \
    OUTPUT_DIR="@(f"{builder_output_dir}")" && \
    mkdir -p /root/.cache/palanteer-git-cache /root/.cache/palanteer-build-cache "$OUTPUT_DIR" && \
    \
    # Clone or update palanteer repository in cache \
    if [ -d /root/.cache/palanteer-git-cache/palanteer ]; then \
        cd /root/.cache/palanteer-git-cache/palanteer && \
        git fetch --depth 1 origin && \
        git reset --hard origin/master; \
    else \
        git clone --depth 1 https://github.com/dfeneyrou/palanteer.git /root/.cache/palanteer-git-cache/palanteer; \
    fi && \
    \
    # Get the current commit hash to detect if we need to rebuild \
    cd /root/.cache/palanteer-git-cache/palanteer && \
    CURRENT_COMMIT=$(git rev-parse HEAD) && \
    BUILD_MARKER="/root/.cache/palanteer-build-cache/build_commit.txt" && \
    \
    # Check if we already built this exact commit \
    if [ -f "$BUILD_MARKER" ] && [ "$(cat $BUILD_MARKER)" = "$CURRENT_COMMIT" ] && [ -d /root/.cache/palanteer-build-cache/bin ]; then \
        echo "Build artifacts for commit $CURRENT_COMMIT already cached, skipping build"; \
    else \
        echo "Building palanteer from commit $CURRENT_COMMIT"; \
        # Copy source to temp location for building \
        cp -r /root/.cache/palanteer-git-cache/palanteer /tmp/palanteer && \
        cd /tmp/palanteer && \
        mkdir -p build && cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make -j$(nproc) && \
        # Cache the build artifacts \
        rm -rf /root/.cache/palanteer-build-cache/bin && \
        cp -r bin /root/.cache/palanteer-build-cache/ && \
        echo "$CURRENT_COMMIT" > "$BUILD_MARKER" && \
        cd /tmp && rm -rf palanteer; \
    fi && \
    \
    # Copy cached build artifacts to output directory \
    cp -r /root/.cache/palanteer-build-cache/bin/* "$OUTPUT_DIR/"'
