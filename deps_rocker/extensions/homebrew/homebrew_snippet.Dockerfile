# syntax=docker/dockerfile:1.4



# Copy Homebrew installation from builder stage
@(f"COPY --from={builder_stage} {builder_output_dir}/.linuxbrew /home/linuxbrew/.linuxbrew")
@(f"COPY --from={builder_stage} {builder_output_dir}/homebrew-env.sh /etc/profile.d/homebrew-env.sh")

# Set up shell environment and fix permissions for all users
RUN chmod 644 /etc/profile.d/homebrew-env.sh && \
    echo 'source /etc/profile.d/homebrew-env.sh' >> /etc/bash.bashrc && \
    echo 'source /etc/profile.d/homebrew-env.sh' >> /root/.bashrc && \
    chmod -R a+w /home/linuxbrew/.linuxbrew

# Set up Homebrew environment variables for Dockerfile build
ENV HOMEBREW_PREFIX=/home/linuxbrew/.linuxbrew
ENV HOMEBREW_CELLAR=/home/linuxbrew/.linuxbrew/Cellar
ENV HOMEBREW_REPOSITORY=/home/linuxbrew/.linuxbrew/Homebrew
ENV PATH="/home/linuxbrew/.linuxbrew/bin:/home/linuxbrew/.linuxbrew/sbin:${PATH}"
ENV MANPATH="/home/linuxbrew/.linuxbrew/share/man:${MANPATH}"
ENV INFOPATH="/home/linuxbrew/.linuxbrew/share/info:${INFOPATH}"
ENV HOMEBREW_NO_ANALYTICS=1
ENV HOMEBREW_NO_AUTO_UPDATE=1
ENV HOMEBREW_ALLOW_ROOT=1
