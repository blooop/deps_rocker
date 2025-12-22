# Add helper function for fzf-powered cd convenience
RUN echo '# fzf setup via pixi tools env' >> ~/.bashrc && \
    echo 'cdfzf() { file=\"$(fzf)\"; [ -n \"$file\" ] && cd \"$(dirname \"$file\")\"; }' >> ~/.bashrc
