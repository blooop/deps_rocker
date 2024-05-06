from setuptools import setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name="deps_rocker",
    version="0.0.5",
    packages=["deps_rocker"],
    package_data={"deps_rocker": ["templates/*.Dockerfile"]},
    author="Austin Gregg-Smith",
    author_email="blooop@gmail.com",
    description="A rocker plugin to help installing apt and pip dependencies",
    long_description=long_description,
    url="https://github.com/blooop/deps_rocker",
    license="MIT",
    install_requires=["rocker", "pyyaml", "toml"],
    # call it odeps_dependencies so that it is called after nvidia "lmnop"
    entry_points={
        "rocker.extensions": [
            "odeps_dependencies = deps_rocker.dependencies:Dependencies",
        ]
    },
)
