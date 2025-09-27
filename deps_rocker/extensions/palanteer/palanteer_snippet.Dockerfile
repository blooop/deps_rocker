# Clone and build palanteer
RUN cd /opt && \
    git clone --depth 1 https://github.com/dfeneyrou/palanteer.git && \
    cd palanteer && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install


# Create symlink to ensure palanteer is accessible
RUN ln -sf /usr/local/bin/palanteer /usr/bin/palanteer || true

# Update PATH to include /usr/local/bin
ENV PATH="/usr/local/bin:${PATH}"
