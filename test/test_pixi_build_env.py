import re

from rocker.core import DockerImageGenerator, RockerExtensionManager

from deps_rocker.pixi_env import DEFAULT_ENV_NAME, get_pixi_collector


def _base_cliargs(**overrides):
    cliargs = {
        "base_image": "ubuntu:24.04",
        "extension_blacklist": [],
        "strict_extension_selection": False,
    }
    cliargs.update(overrides)
    return cliargs


def test_pixi_collector_merges_dependencies():
    collector = get_pixi_collector()
    collector.reset_for_new_run()
    collector.register_contribution(
        extension_name="ext_a", env_name=DEFAULT_ENV_NAME, dependencies={"foo": "1.0"}
    )
    collector.register_contribution(
        extension_name="ext_b", env_name=DEFAULT_ENV_NAME, pypi_dependencies={"bar": "2.0"}
    )

    manifest = collector.render_manifest(env_name=DEFAULT_ENV_NAME)
    assert 'foo = "1.0"' in manifest
    assert 'bar = "2.0"' in manifest
    assert "[environments.tools]" in manifest
    assert "[feature.tools.dependencies]" in manifest


def test_pixi_dockerfile_contains_builder_stage_and_path():
    manager = RockerExtensionManager()
    cliargs = _base_cliargs(pixi=True, npm=True, gitui=True)
    active = manager.get_active_extensions(cliargs)

    pixi_ext = next(ext for ext in active if ext.get_name() == "pixi")
    manifest = pixi_ext.get_files(cliargs)["pixi.toml"]
    assert "nodejs" in manifest
    assert "gitui" in manifest

    dig = DockerImageGenerator(active, cliargs, cliargs["base_image"])
    dockerfile = dig.dockerfile

    assert "FROM ghcr.io/prefix-dev/pixi" in dockerfile
    assert "pixi install" in dockerfile
    assert "-e tools" in dockerfile
    assert "--locked" in dockerfile
    assert re.search(r"/opt/pixi/.pixi/envs/tools/bin", dockerfile)
    assert "COPY pixi.toml" in dockerfile
    assert "COPY --from=pixi_builder" in dockerfile
