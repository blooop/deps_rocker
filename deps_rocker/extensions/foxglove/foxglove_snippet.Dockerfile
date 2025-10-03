# Install Foxglove Studio
RUN apt-get update && apt-get install --no-install-recommends -y \
    libnotify4 \
    xdg-utils \
    libappindicator3-1 \
    && rm -rf /var/lib/apt/lists/*

# Download and install Foxglove Studio .deb package
RUN curl -L -o foxglove-studio-2.15.0-linux-amd64.deb "https://get.foxglove.dev/desktop/latest/foxglove-studio-2.15.0-linux-amd64.deb" \
    && dpkg -i ./foxglove-studio-2.15.0-linux-amd64.deb \
    && rm foxglove-studio-2.15.0-linux-amd64.deb