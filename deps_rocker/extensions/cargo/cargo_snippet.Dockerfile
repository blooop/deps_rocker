# Install cargo from distro packages (fast/simple)
RUN apt-get update && apt-get install -y --no-install-recommends cargo \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

