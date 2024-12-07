import pkgutil
from pathlib import Path
from rocker.extensions import RockerExtension


class VcsTool(RockerExtension):
    name = "vcstool"

    @classmethod
    def get_name(cls):
        return cls.name

    def get_snippet(self, cliargs):
        return pkgutil.get_data("deps_rocker", f"templates/{self.name}_snippet.Dockerfile").decode(
            "utf-8"
        )

    # def get_user_snippet(self, cliargs):
    #     return pkgutil.get_data("deps_rocker", f"templates/{self.name}_user_snippet.Dockerfile").decode(
    #         "utf-8"
    #     )

    def get_files(self, cliargs) -> dict:
        repos = Path.cwd().rglob("*.repos")
        output = {}
        for r in repos:
            if r.is_file():
                with r.open(encoding="utf-8") as f:
                    output[r.name] = f.read()
                    print(r.name)

        return output

    @staticmethod
    def register_arguments(parser, defaults=None):
        if defaults is None:
            defaults = {}
        parser.add_argument(
            f"--{VcsTool.name}",
            action="store_true",
            help=f"add {VcsTool.name} to your docker image",
        )
