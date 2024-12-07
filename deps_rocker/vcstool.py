import pkgutil
from pathlib import Path
from collections import defaultdict
from rocker.extensions import RockerExtension

import em


class VcsTool(RockerExtension):
    name = "vcstool"

    @classmethod
    def get_name(cls):
        return cls.name

    def __init__(self) -> None:
        self.empy_args = dict()
        self.empy_args["depend_repos"] = []

        repos = Path.cwd().rglob("*.repos")
        self.output_files = {}
        for r in repos:
            if r.is_file():
                with r.open(encoding="utf-8") as f:
                    rel_path = r.relative_to(Path.cwd()).as_posix()
                    self.output_files[rel_path] = f.read()
                    self.empy_args["depend_repos"].append(
                        dict(dep=rel_path, path=Path(rel_path).parent.as_posix())
                    )
                    # print(r.name)
                    # print(output[r.name])
        print(self.empy_args)

        super().__init__()

    def get_snippet(self, cliargs):
        snippet = pkgutil.get_data(
            "deps_rocker", f"templates/{self.name}_snippet.Dockerfile"
        ).decode("utf-8")

        print("empy_snippet", snippet)
        print("empy_data", self.empy_args)
        expanded = em.expand(snippet, self.empy_args)

        print("expanded\n", expanded)
        return expanded

    # def get_user_snippet(self, cliargs):
    #     return pkgutil.get_data("deps_rocker", f"templates/{self.name}_user_snippet.Dockerfile").decode(
    #         "utf-8"
    #     )

    def get_files(self, cliargs) -> dict:
        return self.output_files

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{VcsTool.name}",
            action="store_true",
            help=f"add {VcsTool.name} to your docker image",
        )
