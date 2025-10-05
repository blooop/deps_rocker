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

RUN bash -c "set -euxo pipefail && \
        echo 'DEBUG: OUTPUT_DIR=@(f"{builder_output_dir}")' && \
        cmake -S /tmp/palanteer -B /tmp/palanteer/build -DCMAKE_BUILD_TYPE=Release && \
        cmake --build /tmp/palanteer/build --parallel && \
        mkdir -p '@(f"{builder_output_dir}/bin")' && \
        echo 'DEBUG: Copying to @(f"{builder_output_dir}/bin")' && \
        cp /tmp/palanteer/build/bin/* '@(f"{builder_output_dir}/bin")/' && \
        # Build Python wheel if setup.py exists
        if [ -f /tmp/palanteer/python/setup.py ]; then \
            cd /tmp/palanteer/python && python3 setup.py bdist_wheel; \
        fi && \
        mkdir -p '@(f"{builder_output_dir}/dist")' && \
        if compgen -G "/tmp/palanteer/python/dist/*.whl" > /dev/null; then cp /tmp/palanteer/python/dist/*.whl '@(f"{builder_output_dir}/dist")/'; fi && \
        echo 'DEBUG: Listing @(f"{builder_output_dir}")' && \
        ls -lR '@(f"{builder_output_dir}")' && \
        rm -rf /tmp/palanteer"
