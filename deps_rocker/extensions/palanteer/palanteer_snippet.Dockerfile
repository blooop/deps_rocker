
# Clone and build palanteer using BuildKit cache for the git repo (use root-owned cache dir)
RUN --mount=type=cache,target=/root/.cache/palanteer-git-cache \
    cd /tmp && \
    if [ ! -d /root/.cache/palanteer-git-cache/palanteer ]; then \
        git clone --depth 1 https://github.com/dfeneyrou/palanteer.git /root/.cache/palanteer-git-cache/palanteer; \
    fi && \
    cp -r /root/.cache/palanteer-git-cache/palanteer /tmp/palanteer && \
    cd /tmp/palanteer && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    cp bin/* /usr/local/bin/ && \
    cd /tmp && rm -rf palanteer

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
