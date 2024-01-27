import em
import os
import pkgutil
from rocker.extensions import RockerExtension
from pathlib import Path
import yaml
import toml

class Dependencies(RockerExtension):

    name = 'deps_dependencies'

    @classmethod
    def get_name(cls):
        return cls.name

    def precondition_environment(self, cli_args):
        pass

    def read_dependencies(self):
        deps_paths = Path.cwd().glob("dependencies.yaml")
        self.dependencies={}
        for path in deps_paths:
            with open(path, 'r') as file:
                self.dependencies.update(yaml.safe_load(file))

    def get_deps(self,key):
        return " ".join(self.dependencies[key])

    def get_files(self, cliargs):
        all_files = {}

        self.read_dependencies()
        deps_names = ["apt_base","pip_base","apt"]
        for dep in deps_names:
            filename = f"{dep}.deps"
            if dep in self.dependencies:
                all_files[filename] = self.get_deps(dep)
            else:
                all_files[filename] = ""

        all_files["pip.deps"] = self.get_deps("pip") + self.get_pyproject_toml_deps()

        return all_files

    def get_pyproject_toml_deps(self):
        pp_toml = Path.cwd().glob("pyproject.toml")
        deps = []
        for p in pp_toml:
            with open(p, 'r') as f:
                config = toml.load(f)
                project = config["project"]
                if "dependencies" in project:
                    deps.extend(project["dependencies"])
                if "optional-dependencies" in project:
                    optional = project["optional-dependencies"]
                    if "test" in optional:
                        deps.extend(optional["test"])
                    if "dev" in optional:
                        deps.extend(optional["dev"])
                    
        return " "+" ".join(deps)

    def get_preamble(self, cli_args):
        return ''

    def get_snippet(self, cli_args):
        snippet = pkgutil.get_data(
            'deps_rocker',
            'templates/dependencies_snippet.Dockerfile').decode('utf-8')
        return em.expand(snippet)

    def get_docker_args(self, cli_args):
        return ''

    @staticmethod
    def register_arguments(parser,defaults={}):
        parser.add_argument('--deps-dependencies',
            action='store_true',
            help='install dependencies.yaml ')

if __name__ == "__main__":
    res =Dependencies().get_pyproject_toml_deps()
    print(res)
