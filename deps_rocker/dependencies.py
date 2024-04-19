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
        self.empy_args={}
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

    def get_deps(self,key:str,as_list:bool = False)->str:
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
     
        
    def get_files(self, cliargs)->dict:
        self.all_files = {}

        self.read_dependencies()
        #apt and pip deps
        deps_names = ["apt_tools","apt_base","apt","pip_tools","pip_base", "pip"]
        for dep in deps_names:
            self.add_file(dep, self.get_deps(dep))

        #pyproject.toml deps
        self.add_file("pyproject_toml", self.get_pyproject_toml_deps())

        #setup custom scripts
        for s in ["scripts_tools","scripts_base","scripts","scripts_post"]:
            self.add_file(s,self.get_scripts(s))
          
        return self.all_files
    
    def add_file(self,filename:str,content:str)->None:
        """Create a file on the users host machine to be copied into the docker context. This also updates the empy_args dictionary with True if that file is generated.  The empy_args are used to determine if a snipped related to that file should be generated or not.

        Args:
            filename (str): name of file to create
            content (str): the contents of the file
        """
        valid = len(content)>0
        print(f"adding file {filename}, {valid}")
        self.empy_args[filename] =valid
        if valid:
            self.all_files[filename] = content
    
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

        if len(scripts)>1:
            return "\n".join(scripts) 
        return ""
    

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
        
        self.get_files(None)
        self.empy_args["env_vars"] = self.get_deps("env",as_list=True)
        print("all files")
        for k,v in self.all_files.items():
            print(k,v)
        print("empy_args",self.empy_args)
        return em.expand(snippet,self.empy_args)


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

    scr= deps.get_deps("env")
    print(scr)


    # res =Dependencies().get_files(None)


    # print(res)
