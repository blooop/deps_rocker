ENV PATH="${PATH}:/root/.local/bin"
RUN uv tool install specify-cli --from git+https://github.com/github/spec-kit.git \
    && echo 'Spec Kit installed via uv.'
