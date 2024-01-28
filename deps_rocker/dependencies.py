import em
import pkgutil
from rocker.extensions import RockerExtension
from pathlib import Path
import yaml
import toml
from collections import defaultdict

class Dependencies(RockerExtension):

    name = 'deps_dependencies'

    def __init__(self) -> None:
        self.dependencies= defaultdict(set)
        super().__init__()

    @classmethod
    def get_name(cls):
        return cls.name

    def read_dependencies(self):
        """Recursivly load all deps.yaml and create a dictionary containing sets of each type of dependency. Each type of dependency (apt_base, apt etc) should have duplicates rejected when adding to the set"""
       
        for path in Path.cwd().rglob("deps.yaml"):
            with open(path, 'r',encoding="utf-8") as file:
                vals = yaml.safe_load(file)
                for k in vals:                
                    for v in vals[k]:
                        self.dependencies[k].add(v)

    def get_deps(self,key:str)->str:
        """Given a dependency key return a space delimited string of dependencies

        Args:
            key (str): A type of dependency e.g apt

        Returns:
            str: space delimited dependencies
        """
        if key in self.dependencies:
            return " ".join(self.dependencies[key])
        return ""

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

        pip_deps = self.get_deps("pip") +" "+ self.get_pyproject_toml_deps()

        #todo remove this hack
        if pip_deps =="":
            pip_deps = "pip"

        all_files["pip.deps"] =  pip_deps

        return all_files

    def get_pyproject_toml_deps(self)->str:
        """Recursivly load all dependencies from pyproject.toml"

        Returns:
            str: Space delimited string of dependencies
        """
        pp_toml = Path.cwd().rglob("pyproject.toml")
        deps = []
        for p in pp_toml:
            with open(p, 'r',encoding="utf-8") as f:
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
                    
        return " ".join(deps)

    def get_snippet(self, cliargs):
        snippet = pkgutil.get_data(
            'deps_rocker',
            'templates/dependencies_snippet.Dockerfile').decode('utf-8')
        return em.expand(snippet)


    @staticmethod
    def register_arguments(parser,defaults=None):
        parser.add_argument('--deps-dependencies',
            action='store_true',
            help='install deps.yaml ')

if __name__ == "__main__":
    res =Dependencies().get_files(None)
    print(res)
