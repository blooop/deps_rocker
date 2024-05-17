from deps_rocker.dependencies import Dependencies


def gen_dockerfile():
    dep = Dependencies().get_snippet()
    print(dep)
