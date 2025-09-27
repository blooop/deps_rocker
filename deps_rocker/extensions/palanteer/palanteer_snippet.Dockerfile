# Clone and build palanteer
RUN cd /tmp && \
    git clone --depth 1 https://github.com/dfeneyrou/palanteer.git && \
    cd palanteer && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    cp bin/* /usr/local/bin/ && \
    cd /tmp && rm -rf palanteer

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
