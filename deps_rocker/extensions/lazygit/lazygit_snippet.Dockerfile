# syntax=docker/dockerfile:1.4

# lazygit is provided via the shared pixi tools environment
RUN echo "alias lg='lazygit'" >> ~/.bashrc
