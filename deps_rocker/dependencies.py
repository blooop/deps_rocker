from pathlib import Path
from collections import defaultdict
import yaml
import toml
from rocker.extensions import RockerExtension
from .command_layer import CommandLayer


class Dependencies(RockerExtension):
    name = "deps"

    def __init__(self) -> None:
        self.deps_files = []
        self.dependencies = defaultdict(set)
        self.layers_preamble = defaultdict(CommandLayer)
        self.layers = defaultdict(CommandLayer)
        self.layers_user = defaultdict(CommandLayer)
        self.dep_group_order = defaultdict(list)
        self.empy_args = {}
        self.all_files = {}
        self.parsed = False
        super().__init__()

    @classmethod
    def get_name(cls):
        return cls.name

    def setup_deps(self, cliargs):
        if not self.parsed:
            if cliargs is not None:
                file_filter = cliargs["deps"]
                # if ".deps.yaml" not in file_filter:
                # file_filter += ".deps.yaml"
            else:
                file_filter = "*.deps.yaml"
            self.read_dependencies(Path.cwd(), file_filter)
            self.parsed = True

    def read_dependencies(self, path: Path, pattern: str):
        """Recursivly load all deps.yaml and create a dictionary containing sets of each type of dependency. Each type of dependency (apt_base, apt etc) should have duplicates rejected when adding to the set"""
        for p in path.rglob(pattern):
            print(f"found {p}")
            self.deps_files.append(p)
            with open(p, "r", encoding="utf-8") as file:
                vals = yaml.safe_load(file)
                print(vals)
                if vals is not None:
                    prev_k = None
                    for k in vals:
                        for v in vals[k]:
                            if "script" in k:
                                if v is not None:
                                    v = (p.parent / Path(v)).absolute().as_posix()
                            print(f"key:{k} val:{v}")

                            if "preamble" in k:
                                self.layers_preamble[k].update(k, v)
                            elif "user" in k:
                                self.layers_user[k].update(k, v)
                            else:
                                self.layers[k].update(k, v)

                            self.dependencies[k].add(v)
                        if prev_k is not None:
                            self.dep_group_order[k].append(prev_k)
                        prev_k = k

        for k, v in self.dep_group_order.items():
            self.dep_group_order[k] = list(dict.fromkeys(v))

    def get_deps(self, key: str, as_list: bool = False) -> str:
        """Given a dependency key return a space delimited string of dependencies

        Args:
            key (str): A type of dependency e.g apt

        Returns:
            str: space delimited dependencies
        """
        if key in self.dependencies:
            dep_list = list(sorted(self.dependencies[key]))
            if as_list:
                return dep_list
            return " ".join(dep_list)
        return ""

    def get_files(self, cliargs) -> dict:
        """Get a dict of local filenames and content to write into them"""
        print("Getting files")

        self.add_file("pyproject_default", self.get_pyproject_toml_deps())

        all_layers = (
            list(self.layers_preamble.values())
            + list(self.layers.values())
            + list(self.layers_user.values())
        )

        for lay in all_layers:
            print(lay.get_filename())
            if "script" in lay.command:
                fn = lay.get_filename()
                self.add_file(fn, self.get_scripts(fn))

        return self.all_files

    def add_file(self, filename: str, content: str) -> None:
        """Create a file on the users host machine to be copied into the docker context. This also updates the empy_args dictionary with True if that file is generated.  The empy_args are used to determine if a snipped related to that file should be generated or not.

        Args:
            filename (str): name of file to create
            content (str): the contents of the file
        """
        valid = len(content) > 0
        print(f"adding file {filename}, {valid}")
        self.empy_args[filename] = valid
        if valid:
            self.all_files[filename] = content

    def get_scripts(self, name: str) -> str:
        """collect all scripts files into a single script

        Args:
            name (str): name of the scripts key and output script name
        Returns:
            str: All scripts combined into a single script
        """
        # make sure the first line has shebang
        scripts = ["#! /bin/bash"]
        scripts_deps = self.get_deps(name)
        if len(scripts_deps) > 0:
            for s in scripts_deps.split(" "):
                with open(s, encoding="utf-8") as f:
                    scripts.extend(f.readlines())

        if len(scripts) > 1:
            combined = "\n".join(scripts)
            combined = combined.replace("sudo ", "")
            combined = combined.replace("sudo", "")
            return combined
        return ""

    def get_pyproject_toml_deps(self) -> str:
        """Recursivly load all dependencies from pyproject.toml"

        Returns:
            str: Space delimited string of dependencies
        """
        pp_toml = Path.cwd().rglob("pyproject.toml")
        pyproj_deps = []
        for p in pp_toml:
            with open(p, "r", encoding="utf-8") as f:
                config = toml.load(f)
                if "project" in config:
                    project = config["project"]
                    if "dependencies" in project:
                        pyproj_deps.extend(project["dependencies"])
                    if "optional-dependencies" in project:
                        optional = project["optional-dependencies"]
                        if "test" in optional:
                            pyproj_deps.extend(optional["test"])
                        if "dev" in optional:
                            pyproj_deps.extend(optional["dev"])
        return " ".join(pyproj_deps)

    def get_preamble(self, cliargs):
        return "\n".join([lay.to_snippet() for lay in self.layers_preamble.values()])

    def get_snippet(self, cliargs=None):
        self.setup_deps(cliargs)
        return "\n".join([lay.to_snippet() for lay in self.layers.values()])

    def get_user_snippet(self, cliargs):
        """Get a dockerfile snippet to be executed after switchingto the expected USER."""
        self.setup_deps(cliargs)
        return "\n".join([lay.to_snippet() for lay in self.layers_user.values()])

    @staticmethod
    def register_arguments(parser, defaults=None):
        # parser.add_argument("--deps", action="store_true", help="install all deps.yaml ")
        parser.add_argument(
            "--deps",
            type=str,
            nargs="?",
            const="*.deps.yaml",
            help="A filter to select deps.yaml files. Defaults to *.deps.yaml",
        )


if __name__ == "__main__":
    deps = Dependencies()
    # print(deps)

    # scr= deps
    # scr= deps.get_scripts("scripts_tools")
    scr = deps.get_deps("scripts")

    scr = deps.get_deps("env")
    deps.get_snippet(None)
    print(scr)

    # res =Dependencies().get_files(None)

    # print(res)
