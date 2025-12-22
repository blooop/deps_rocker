# Install Google Gemini CLI tool via Homebrew (must run as non-root user)
# Note: This runs in the root section but uses su to run as non-root
RUN su - $(getent passwd 1000 | cut -d: -f1) -c 'brew install gemini-cli'
