from deps_rocker.simple_rocker_extension import SimpleRockerExtension


class GitClone(SimpleRockerExtension):
    """Adds support for git cloning"""

    name = "git_clone"

    # deps = ["git"]

    # def invoke_after(self, cliargs):
    #     return set(["git"])

    # def required(self, cliargs):
    #     return set(["git"])
