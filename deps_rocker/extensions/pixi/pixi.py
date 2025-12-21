from deps_rocker.simple_rocker_extension import (
    SimpleRockerExtension,
    consume_npm_global_packages,
    consume_pixi_global_packages,
)
import json
from typing import Dict, List


class Pixi(SimpleRockerExtension):
    """Install pixi and enable shell completion"""

    name = "pixi"
    depends_on_extension: tuple[str, ...] = ("user",)
    builder_apt_packages = ["curl", "ca-certificates"]

    # Template arguments for both snippets
    empy_args = {
        "PIXI_VERSION": "0.62.1",
    }

    @staticmethod
    def _runtime_entrypoint() -> str:
        return """#!/bin/bash
set -e

# Prefer a user-writable pixi home so runtime sync can populate caches
DEFAULT_PIXI_HOME="${HOME}/.pixi"
if [ -n "${PIXI_HOME}" ] && [ -w "${PIXI_HOME}" ]; then
    ACTIVE_PIXI_HOME="${PIXI_HOME}"
else
    ACTIVE_PIXI_HOME="${DEFAULT_PIXI_HOME}"
fi

export PIXI_HOME="${ACTIVE_PIXI_HOME}"
export PATH="${PIXI_HOME}/bin:${PATH}"

mkdir -p "${PIXI_HOME}"
if [ -z "$(ls -A "${PIXI_HOME}" 2>/dev/null)" ] && [ -d /opt/deps_rocker/pixi ]; then
    cp -a /opt/deps_rocker/pixi/. "${PIXI_HOME}/"
fi

manifest_src=/etc/deps-rocker/pixi-global.toml
manifest_dest="${PIXI_HOME}/manifests/pixi-global.toml"
if [ -f "${manifest_src}" ]; then
    mkdir -p "$(dirname "${manifest_dest}")"
    cp "${manifest_src}" "${manifest_dest}"
fi

# Sync global Pixi dependencies into mounted caches
pixi global sync

npm_globals_file=/etc/deps-rocker/npm-globals.txt
if [ -s "${npm_globals_file}" ] && command -v npm >/dev/null 2>&1; then
    while IFS= read -r pkg; do
        [ -z "${pkg}" ] && continue
        if ! npm list -g "${pkg}" >/dev/null 2>&1; then
            npm install -g "${pkg}"
        fi
    done < "${npm_globals_file}"
fi

echo "Extension test complete"

# Execute the user command
exec "$@"
"""

    @staticmethod
    def _global_envs() -> List[Dict]:
        """Collect all global packages registered by enabled extensions."""
        return consume_pixi_global_packages()

    def _format_manifest(self) -> str:
        """Build a pixi-global manifest TOML content."""
        lines: list[str] = ["version = 1", ""]
        for env in self._global_envs():
            lines.append(f"[envs.{env['name']}]")
            lines.append(f"channels = {json.dumps(env['channels'])}")
            dep_items = ", ".join([f'{k} = "{v}"' for k, v in sorted(env["deps"].items())])
            lines.append(f"dependencies = {{ {dep_items} }}")
            exposed = env.get("exposed") or {}
            if exposed:
                expose_items = ", ".join([f'{k} = "{v}"' for k, v in sorted(exposed.items())])
                lines.append(f"exposed = {{ {expose_items} }}")
            lines.append("")
        return "\n".join(lines).rstrip() + "\n"

    def get_files(self, cliargs) -> dict[str, str]:
        """Provide manifest and entrypoint files for runtime pixi sync."""
        npm_globals = consume_npm_global_packages()
        npm_globals_content = ""
        if npm_globals:
            npm_globals_content = "\n".join(sorted(npm_globals)) + "\n"

        return {
            "pixi-global.toml": self._format_manifest(),
            "deps_rocker_entrypoint.sh": self._runtime_entrypoint(),
            "npm-globals.txt": npm_globals_content,
        }
