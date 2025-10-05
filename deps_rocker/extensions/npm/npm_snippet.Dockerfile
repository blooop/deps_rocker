# Install nvm, node, and npm from builder stage artifacts
ENV NVM_DIR=/usr/local/nvm
ENV NODE_VERSION=24.9.0
ENV NPM_VERSION=11.6.1

COPY --from=@builder_stage@ @builder_output_dir@/nvm $NVM_DIR
COPY --from=@builder_stage@ @builder_output_dir@/nvm-env.sh /etc/profile.d/nvm-env.sh
RUN chmod 644 /etc/profile.d/nvm-env.sh && \
    echo '. /etc/profile.d/nvm-env.sh' >> /etc/bash.bashrc && \
    echo '. /etc/profile.d/nvm-env.sh' >> /root/.bashrc

RUN bash -lc '. /etc/profile.d/nvm-env.sh && echo "Installed npm version:" && npm --version'

ENV PATH="$NVM_DIR/versions/node/v$NODE_VERSION/bin:$PATH"
