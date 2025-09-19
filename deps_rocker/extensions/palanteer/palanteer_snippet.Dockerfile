RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential libgl1-mesa-dev libxrender-dev git cmake ca-certificates python3-dev python3-setuptools python3-wheel \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/dfeneyrou/palanteer.git \
    && cd palanteer && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j"$(nproc)" install \
    && cp /palanteer/build/bin/palanteer /usr/local/bin/ \
    && cd / && rm -rf palanteer

