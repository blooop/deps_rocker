# Install Google Gemini CLI tool via npm with BuildKit cache for global installs
RUN --mount=type=cache,target=/root/.npm,id=global-npm-cache \
	npm install -g @@google/gemini-cli
