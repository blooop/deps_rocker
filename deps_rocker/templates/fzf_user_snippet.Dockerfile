# Install fzf from source as apt is very out of date
RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf;  ~/.fzf/install --all