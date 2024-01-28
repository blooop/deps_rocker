import em
import pkgutil
from rocker.extensions import RockerExtension
from pathlib import Path
import yaml
import toml
from deps_baseclass import DependenciesBase
class AptDependencies(DependenciesBase):

    name = 'deps_apt'

    @classmethod
    def get_name(cls):
        return cls.name

    def get_files(self, cliargs=None):
        all_files = {}

        self.read_dependencies()

        for d in self.dependencies:
            if "apt" in d:
                print(d)

        # deps_names = ["apt_base","pip_base","apt"]
        # for dep in deps_names:
        #     filename = f"{dep}.deps"
        #     if dep in self.dependencies:
        #         all_files[filename] = self.get_deps(dep)
        #     else:
        #         all_files[filename] = ""



        return all_files

    def get_snippet(self, cliargs):
        snippet = pkgutil.get_data(
            'deps_rocker',
            'templates/dependencies_snippet.Dockerfile').decode('utf-8')
        return em.expand(snippet)

    @staticmethod
    def register_arguments(parser,defaults={}):
        parser.add_argument('--deps-apt',
            action='store_true',
            help='install deps.yaml ')

if __name__ == "__main__":
    res =AptDependencies().get_files()
    print(res)
