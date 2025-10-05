# Install fzf from staged source
RUN rm -rf ~/.fzf && mkdir -p ~/.fzf && cp -a /opt/deps_rocker/fzf/. ~/.fzf/
RUN ~/.fzf/install --all

# Add cdfzf function to bashrc
RUN echo 'cdfzf() { file="$(fzf)"; [ -n "$file" ] && cd "$(dirname "$file")"; }' >> ~/.bashrc
