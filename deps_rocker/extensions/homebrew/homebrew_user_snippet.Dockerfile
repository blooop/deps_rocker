# syntax=docker/dockerfile:1.4

# Set up Homebrew environment for user
RUN echo 'source /etc/profile.d/homebrew-env.sh' >> ~/.bashrc
