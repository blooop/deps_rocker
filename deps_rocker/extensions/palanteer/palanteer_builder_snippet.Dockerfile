@(f"FROM {base_image} AS {builder_stage}")

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked,id=apt-cache \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked,id=apt-lists \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        python3-dev \
        python3-pip \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libx11-dev \
        libxrandr-dev \
        libxinerama-dev \
        libxcursor-dev \
        libxi-dev \
    && rm -rf /var/lib/apt/lists/*

ADD https://github.com/dfeneyrou/palanteer.git#main /tmp/palanteer

RUN set -euxo pipefail && \
    OUTPUT_DIR="@builder_output_dir@" && \
    cmake -S /tmp/palanteer -B /tmp/palanteer/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/palanteer/build --parallel && \
    mkdir -p "$OUTPUT_DIR/bin" && \
    cp /tmp/palanteer/build/bin/* "$OUTPUT_DIR/bin/" && \
    rm -rf /tmp/palanteer
