import pkgutil
from pathlib import Path
import em
from rocker.extensions import RockerExtension
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class VcsTool(SimpleRockerExtension):
    name = "vcstool"

    def __init__(self) -> None:
        self.empy_args = dict()
        self.empy_args["depend_repos"] = []
        self.output_files = self.generate_files()
        super().__init__()

    def generate_files(self):
        """Generates depend.repos files and collects their paths

        Returns:
            dict[str]: _description_
        """
        repos = Path.cwd().rglob("*.repos")
        output_files = {}
        for r in repos:
            if r.is_file():
                with r.open(encoding="utf-8") as f:
                    rel_path = r.relative_to(Path.cwd()).as_posix()
                    output_files[rel_path] = f.read()
                    self.empy_args["depend_repos"].append(
                        dict(dep=rel_path, path=Path(rel_path).parent.as_posix())
                    )
        return output_files

    def get_snippet(self, cliargs):
        snippet = pkgutil.get_data(
            "deps_rocker", f"templates/{self.name}_snippet.Dockerfile"
        ).decode("utf-8")

        print("empy_snippet", snippet)
        print("empy_data", self.empy_args)
        expanded = em.expand(snippet, self.empy_args)

        print("expanded\n", expanded)
        return expanded

    def get_files(self, cliargs) -> dict:
        return self.output_files

    def invoke_after(self, cliargs):
        return set(["cwd"])

    @staticmethod
    def register_arguments(parser, defaults=None):
        SimpleRockerExtension.register_arguments_helper(VcsTool.name, parser, defaults)