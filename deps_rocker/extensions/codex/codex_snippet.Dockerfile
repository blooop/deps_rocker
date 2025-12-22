# Install OpenAI Codex CLI with BuildKit cache for npm global installs
RUN --mount=type=cache,target=/root/.npm,id=global-npm-cache \
	npm install -g @@openai/codex
