# Install Foxglove Studio via direct download
RUN FOXGLOVE_VERSION="2.34.0" && \
    FOXGLOVE_DEB="foxglove-studio-${FOXGLOVE_VERSION}-linux-amd64.deb" && \
    FOXGLOVE_URL="https://get.foxglove.dev/desktop/v${FOXGLOVE_VERSION}/${FOXGLOVE_DEB}" && \
    echo "Downloading Foxglove Studio ${FOXGLOVE_VERSION}..." && \
    curl --retry 3 --retry-delay 5 --progress-bar -L -o "${FOXGLOVE_DEB}" "${FOXGLOVE_URL}" && \
    echo "Verifying package integrity..." && \
    dpkg-deb --info "${FOXGLOVE_DEB}" > /dev/null 2>&1 && \
    dpkg -i "${FOXGLOVE_DEB}" && \
    rm "${FOXGLOVE_DEB}" && \
    rm -f /etc/apt/sources.list.d/foxglove-studio.list