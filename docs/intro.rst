Intro
=====

To set up your project run the vscode task "pull updates from template repo" and then task "rename project template name"

## Installation

```
pip install deps-rocker
```

## Usage

```
#recursively search for *.deps.yaml and install those packages on top of an existing image
rocker --deps ubuntu:22.04  
```

## Motivation

Docker enables easy isolation of dependencies from the host system, but it is not easy to dynamically combine docker files from separate projects into a single unified environment.


## Intro

This is a [rocker](https://github.com/tfoote/rocker) extension for automating dependency installation.  The aim is to allow a projects to define its development dependencies in a deps.yaml file which are added to the rocker container.  If two projects define their dependencies in separate files, the extension will combine the common commands into the same docker layer to help reduce image size and duplication of work.

For example:

pkg a requires git, make and ffmpeg and pkg_b requires git-lfs and pip.  Their deps.yaml files would look something like: 

pkg_a.deps.yaml:

```
apt_sources:
  - git

apt_language-toolchain:
  - make
  - gcc

apt:
  - ffmpeg
```
pkg_b.deps.yaml

```
apt_sources:
  - git
  - git-lfs

apt_language-toolchain:
  - python3-pip
```

If you wanted a container that had the dependencies of both installed deps-rocker would combine the dependencies to produce a file like:

```
apt_sources:
  - git
  - git-lfs

apt_language-toolchain:
  - make
  - gcc
  - python3-pip

apt:
  - ffmpeg
```

Each heading in the yaml file produces a docker layer based on the command and the label.  The format of the labels is {command_name}_{command-label}.  The layer names are delimited by _ so layer names should use - eg: language-toolchain. 

This makes it easy to define the dependencies for a single project, but enable reuse of common dependencies across multiple projects. However, deps rocker does not restrict what is defined in each layer and so relies on a common convention for multiple packages to play nicely with each other.  If one package adds "make" to apt_sources and other package adds "make" to apt_langage_toolchain, the deps-rocker will not complain and will not deduplicate that install step.   

## Methodology:

The algorithm works by splitting each entry in the yaml file into a command and a layer.  The entries from all the deps.yaml files are grouped by the command and layer into a list of entries for that command.  The order of the commands is defined by the order they appear in the deps.yaml file.  As long as all the files follow the same order of commands then a dependency tree of commands can be created and executed in a deterministic order.  However if two files define conflicting orders deps-rocker will not be able to produce a deterministic set of commands and fail.  e.g:

pkg_a.deps.yaml:

```
apt_sources:
  - git

apt_language-toolchain:
  - make
  - gcc
```
pkg_b.deps.yaml

```
apt_language-toolchain:
  - python3-pip

apt_sources:
  - git
  - git-lfs
```

pkg_a says that apt_langage-toolchain comes before apt_sources, and pkg_b says that apt_sources comes before apt_language-toolchain, which is a conflict. 

The pseudocode for the deps-rocker algorithm is as follows:
```
dependencies_dictionary
for file in glob(*.deps.yaml):
  for entry in file.entries:
    add 
```

If two packages have unique layers that depend on a common layer

pkg_a.deps.yaml:

```
apt_sources:
  - git

apt_pkg_a_custom:
  - custom1
```
pkg_b.deps.yaml

```
apt_sources:
  - git-lfs

apt_pkg_b_custom:
  - custom1
```

Here apt_pkg_b_custom and apt_pkg_a_custom both need to be run after apt_sources.  They will be run run in alphabetical order (to ensure determinism)


## Commands

Commands are defined in templates/commandname_snippet.Dockerfile.

They use the [empy](https://pypi.org/project/empy/) templating language that is used by [rocker](https://github.com/tfoote/rocker).  deps-rocker has some basic commands already implemented but adding a new command is as simple as adding a _snippet.Dockerfile.  

Existing Commands:
  - apt: apt install packages
  - add-apt-repository: add repositories to apt
  - env: define environment variables
  - pip: install pip packages
  - run: RUN a docker command
  - script: run a script.
  - pyproject: look for any local pyproject.toml files and install dependencies listed there. 


script:

If you have sudo inside your script deps-rocker will automatically remove them.  This is so that you can run the script on the host machine where sudo is required. 

## Layer conventions

As mentioned above, deps-rocker does not enforce any particular layer order so the user can define them as they see fit, however to enhance interoperation of packages we define a suggested layer order.  Examples of deps.yaml can be found in [manifest_rocker](https://github.com/blooop/manifest_rocker/tree/all/pkgs)

the template_pkg has common layers and dependencies that go in each layer as a guide to maximise reusability and caching.
[template_pkg](https://github.com/blooop/manifest_rocker/blob/main/pkgs/template_pkg/template_pkg.deps.yaml)

```
# Template package  Uncomment or modify these entries.

env_base:
  - DEPS_ROCKER=1

apt_base: #lowest level of dependency that changes very infrequently
  - build-essential

apt_io: #graphics sound, input devices etc
  - libasound2

apt_sources: #apt dependencies for setting up software sources
  - ca-certificates #needed for wget
  - wget
  - curl
  - lsb-release
  - gnupg2
  - git
  - git-lfs

script_sources: #scripts for adding repositories or repo keys
  - sources.sh

apt_language-toolchain: #packages related to setting up languages e.g. c++,python,rust etc
  - python3-pip
  - make
  - gcc

pip_language-toolchain: #install basic development tools which almost never change
  - pip #this updates pip to latest version
  - flit
  - pytest
  - ruff

apt_tools: #any other development tools
  - colcon

apt: #the main dependencies of the package
  - fsearch

pyproject: #Scan for all pyproject.tomls and install
  - all

script_build: #any build steps
  - build.sh

script_lint: 
  - lint.sh

script_test:
  - test.sh


## limitations/TODO

This has only been tested on the ubuntu base image. It assumes you have access to apt-get.
