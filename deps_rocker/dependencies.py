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
        self.read_dependencies()
        print("dependencies dictionary")
        for k,v in self.dependencies.items():
            print(k,v)
        super().__init__()

    @classmethod
    def get_name(cls):
        return cls.name

    def read_dependencies(self):
        """Recursivly load all deps.yaml and create a dictionary containing sets of each type of dependency. Each type of dependency (apt_base, apt etc) should have duplicates rejected when adding to the set"""
        for path in Path.cwd().rglob("deps.yaml"):
            print(f"found {path}")
            with open(path, 'r',encoding="utf-8") as file:
                vals = yaml.safe_load(file)
                for k in vals:                
                    for v in vals[k]:
                        if "scripts" in k:
                            v = (path.parent/ Path(v)).absolute().as_posix()
                        print(k,v)
                        self.dependencies[k].add(v)

    def get_deps(self,key:str)->str:
        """Given a dependency key return a space delimited string of dependencies

        Args:
            key (str): A type of dependency e.g apt

        Returns:
            str: space delimited dependencies
        """
        if key in self.dependencies:
            return " ".join(sorted(self.dependencies[key]))
        return ""
    
    def get_pips_deps(self,key:str)->str:
        pip_deps = self.get_deps(key)
        print(pip_deps)

        #todo remove this hack
        if pip_deps =="":
            pip_deps = "pip"
        return pip_deps
        
    def get_files(self, cliargs)->dict:
        all_files = {}

        self.read_dependencies()
        #apt deps
        deps_names = ["apt_tools","apt_base","apt"]
        for dep in deps_names:
            all_files[f"{dep}.deps"]= self.get_deps(dep)

        #pip deps
        deps_names = ["pip_tools","pip_base"]
        for dep in deps_names:
            all_files[f"{dep}.deps"]=self.get_pips_deps(dep) 

        all_files["pip.deps"] =  self.get_pips_deps("pip") +" "+ self.get_pyproject_toml_deps()

        #setup custom scripts
        all_files["scripts_tools.sh"] = self.get_scripts("scripts_tools")
        all_files["scripts_base.sh"] = self.get_scripts("scripts_base")
        all_files["scripts.sh"] = self.get_scripts("scripts")
        all_files["scripts_post.sh"] = self.get_scripts("scripts_post")

        all_files["env_vars"] = self.get_deps("env")

        return all_files
    
    def get_scripts(self,name:str)->str:
        """collect all scripts files into a single script

        Args:
            name (str): name of the scripts key and output script name
        Returns:
            str: All scripts combined into a single script
        """
        #make sure the first line has shebang
        scripts = ["#! /bin/bash"]
        scripts_deps = self.get_deps(name)
        if len(scripts_deps)>0:
            for s in scripts_deps.split(" "):
                with open(s,encoding="utf-8") as f:
                    scripts.extend(f.readlines())

        return "\n".join(scripts) 
    

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

    deps = Dependencies()
    # print(deps)

    # scr= deps
    # scr= deps.get_scripts("scripts_tools")
    scr= deps.get_deps("scripts")

    # print(scr)


    # res =Dependencies().get_files(None)


    # print(res)
