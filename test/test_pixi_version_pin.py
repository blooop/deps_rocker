from deps_rocker.extensions.pixi.pixi import Pixi


def test_builder_stage_sees_pixi_version():
    """An ARG declared before FROM is not visible to RUN in that stage."""
    snippet = Pixi().get_builder_snippet({})
    lines = [line.strip() for line in snippet.splitlines()]
    from_index = next(i for i, line in enumerate(lines) if line.startswith("FROM "))
    install_index = next(i for i, line in enumerate(lines) if "install.sh" in line)
    stage_lines = lines[from_index:install_index]
    assert any(line.split("=")[0] == "ARG PIXI_VERSION" for line in stage_lines)
