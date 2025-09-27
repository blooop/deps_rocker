# Clone and build palanteer
RUN cd /tmp && \
    git clone --depth 1 https://github.com/dfeneyrou/palanteer.git && \
    cd palanteer && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    cd / && rm -rf /tmp/palanteer

# Install palanteer Python package
RUN pip3 install palanteer==0.8.0