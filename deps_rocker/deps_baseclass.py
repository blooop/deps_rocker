import em
import pkgutil
from rocker.extensions import RockerExtension
from pathlib import Path
import yaml
import toml

class DependenciesBase(RockerExtension):
   
    def read_dependencies(self):
        deps_paths = Path.cwd().glob("deps.yaml")
        self.dependencies={}
        for path in deps_paths:
            with open(path, 'r') as file:
                self.dependencies.update(yaml.safe_load(file))

    def get_deps(self,key)->str:
        if key in self.dependencies:
            return " ".join(self.dependencies[key])
        return ""
    
    def get_preamble(self, cliargs):
        return ''
    
    def get_docker_args(self, cliargs):
        return ''