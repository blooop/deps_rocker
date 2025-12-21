from deps_rocker.extensions.pixi.pixi import Pixi
from deps_rocker.simple_rocker_extension import (
    consume_pixi_global_packages,
    consume_npm_global_packages,
    register_npm_global_packages,
    register_pixi_global_packages,
)


def test_pixi_manifest_uses_registered_packages():
    # Reset registry for an isolated run
    consume_pixi_global_packages()

    register_pixi_global_packages(
        "git_clone",
        [
            {
                "name": "git",
                "channels": ["conda-forge"],
                "deps": {"git": "*"},
                "exposed": {"git": "git"},
            }
        ],
    )
    register_pixi_global_packages(
        "npm",
        [
            {
                "name": "nodejs",
                "channels": ["conda-forge"],
                "deps": {"nodejs": "*"},
                "exposed": {"node": "node"},
            }
        ],
    )
    register_npm_global_packages(["@openai/codex", "@google/gemini-cli"])

    files = Pixi().get_files({"pixi": True})
    manifest = files["pixi-global.toml"]

    assert "[envs.git]" in manifest
    assert 'dependencies = { git = "*"' in manifest
    assert "[envs.nodejs]" in manifest
    assert 'exposed = { node = "node"' in manifest
    assert "@openai/codex" in files["npm-globals.txt"]
    assert "@google/gemini-cli" in files["npm-globals.txt"]

    # Registry is drained after manifest generation
    assert consume_pixi_global_packages() == []
    assert consume_npm_global_packages() == []
