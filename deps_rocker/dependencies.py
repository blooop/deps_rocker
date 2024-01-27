import em
import os
import pkgutil
from rocker.extensions import RockerExtension
from pathlib import Path
import yaml

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
        deps_names = ["apt_base","pip_base","apt","pip"]
        for dep in deps_names:
            filename = f"{dep}.deps"
            if dep in self.dependencies:
                all_files[filename] = self.get_deps(dep)
            else:
                all_files[filename] = ""

        return all_files

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

