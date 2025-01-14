from pathlib import Path
from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class Setuppy(SimpleRockerExtension):
    """Searches for all setup.py files and performs and editable install"""

    name = "setuppy"

    def invoke_after(self, cliargs) -> set:
        return {"user", "cwd"}

    def required(self, cliargs):
        return {"user", "cwd"}

    def find_setuppy(self):
        """Recursively load all dependencies from pyproject.toml"

        Returns:
            str: Space delimited string of dependencies
        """
        setuppys = Path.cwd().rglob("setup.py")

        setuppys_paths = []
        for s in setuppys:
            setuppys_paths.append(
                (Path("/workspaces") / s.relative_to(Path.cwd()).parent).as_posix()
            )
        return setuppys_paths

    def get_user_snippet(self, cliargs):
        """Get a dockerfile snippet to be executed after switching to the expected USER."""
        setuppytes = self.find_setuppy()
        self.empy_user_args = {"data_list": setuppytes}
        tmp = super().get_user_snippet(cliargs)
        print(tmp)
        return tmp
        # return "\n".join([lay.to_snippet() for lay in self.layers_user.values()])
