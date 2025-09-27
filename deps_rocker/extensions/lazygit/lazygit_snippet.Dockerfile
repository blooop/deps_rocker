RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates git \
&& apt-get clean && rm -rf /var/lib/apt/lists/*

RUN LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" | grep -Po '"tag_name": "v\K[^"]*') \
&& echo "Lazygit version: ${LAZYGIT_VERSION}" \
&& curl -Lo lazygit.tar.gz -L "https://github.com/jesseduffield/lazygit/releases/download/v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" \
&& ls -lh lazygit.tar.gz \
&& tar -xzf lazygit.tar.gz lazygit \
&& install lazygit /usr/local/bin \
&& rm lazygit.tar.gz lazygit
