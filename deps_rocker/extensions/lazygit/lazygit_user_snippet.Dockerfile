# Install lazygit via pixi
RUN pixi global install lazygit

# Add an alias for lazygit to .bashrc
RUN echo "alias lg='lazygit'" >> ~/.bashrc
