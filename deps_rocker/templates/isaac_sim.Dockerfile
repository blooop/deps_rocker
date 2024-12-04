# RUNNING SCRIPT: @layer_name

COPY @filename /@filename
RUN bash /@filename

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip cmake build-essential && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip install isaacsim==4.2.0.2 


# isaacsim-extscache-kit = ">=4.2.0.2, <5"
# isaacsim-extscache-kit-sdk = ">=4.2.0.2, <5"
# isaacsim-extscache-physics = ">=4.2.0.2, <5"