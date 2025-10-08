# Use an official Ubuntu base image
FROM ubuntu:22.04
# Install latest stable Neovim via PPA
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository ppa:neovim-ppa/stable && \
    apt-get update && \
    apt-get install -y neovim && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Ensure config directories are present
RUN mkdir -p /root/.config/nvim /root/.vim
