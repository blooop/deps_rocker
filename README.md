# template_rocker

## Continuous Integration Status

[![Ci](https://github.com/blooop/template_rocker/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/blooop/template_rocker/actions/workflows/ci.yml?query=branch%3Amain)
[![Codecov](https://codecov.io/gh/blooop/template_rocker/branch/main/graph/badge.svg?token=Y212GW1PG6)](https://codecov.io/gh/blooop/template_rocker)
[![GitHub issues](https://img.shields.io/github/issues/blooop/template_rocker.svg)](https://GitHub.com/blooop/template_rocker/issues/)
[![GitHub pull-requests merged](https://badgen.net/github/merged-prs/blooop/template_rocker)](https://github.com/blooop/template_rocker/pulls?q=is%3Amerged)
[![GitHub release](https://img.shields.io/github/release/blooop/template_rocker.svg)](https://GitHub.com/blooop/template_rocker/releases/)
[![License](https://img.shields.io/github/license/blooop/template_rocker
)](https://opensource.org/license/mit/)
[![Python](https://img.shields.io/badge/python-3.8%20%7C%203.9%20%7C%203.10%20%7C%203.11%20%7C%203.12-blue)](https://www.python.org/downloads/)
[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

## Intro

This a template for creating [rocker](https://github.com/tfoote/rocker) extensions.  It used [off-your-rocker](https://github.com/sloretz/off-your-rocker) as a starting point, and migrated to use `pyproject.toml` instead of `setup.py` to help manage the development environment and continuous integration in a modern way using [pixi](https://pixi.sh).  [Pixi](https://github.com/prefix-dev/pixi) is a cross-platform package manager based on the conda ecosystem.  It provides a simple and performant way of reproducing a development environment and running user defined tasks and worflows.  It more lightweight than docker, but does not provide the same level of isolation or generality.

This has basic setup for

* pylint
* ruff
* black
* pytest
* git-lfs
* basic github actions ci
* pulling updates from this template
* codecov
* pypi upload
* dependabot

# Install

1. Use github to use this project as a template
2. Clone the project and run, `scripts/update_from_template.sh` and then run the `scripts/rename_project.sh` to rename the project.

If you are using pixi, look at the available tasks in pyproject.toml.  If you are new to pixi follow the instructions on the pixi [website](https://prefix.dev/)

# Github setup

There are github workflows for CI, codecov and automated pypi publishing in `ci.yml` and `publish.yml`.

ci.yml uses pixi tasks to set up the environment matrix and run the various CI tasks. To set up codecov on github, you need to get a `CODECOV_TOKEN` and add it to your actions secrets.

publish.yml uses [pypy-auto-publish](https://github.com/marketplace/actions/python-auto-release-pypi-github) to automatically publish to pypi if the package version number changes. You need to add a `PYPI_API_TOKEN` to your github secrets to enable this.     

If you use vscode to attach to your development container it makes it easier to set up specific extensions for each project that don't need to be installed globally. 

# Development

There are currently two ways of running code.  The preferred way is to use pixi to manage your environment and dependencies. 

```bash
$ cd project

$ pixi run ci
$ pixi run arbitrary_task
```

# Adding Functionality

1. Rename template_rocker/new_rocker_extension.py and the class inside to something more appropriate
2. Update the extension entrypoint in `pyproject.toml` 

    `[project.entry-points."rocker.extensions"]`

    `new_rocker_extension = "template_rocker.new_rocker_extension:NewRockerExtension"`

    
    e.g.:
    `lazygit = "lazygit_rocker.lazygit_rocker:LazygitExtension"`

    
3. Update the `get_name()` function in the extension class. e.g. Updating the name to `lazygit` would mean that it would be activated by passing --lazygit to rocker   
   
4. Add/update the docker snippets in the templates folder.  Officially the extension is supposed to be .Dockerfile.em to indicate it's passed to the empy library, but I have left them as .Dockerfile as I get Dockerfile highlighting out of the box that way. 
5. Develop your extension.  I find that using `pip install -e .` and running the extension with rocker on my host system is the easiest way to get started.  

    ```bash
    rocker --lazygit ubuntu:22.04

    #check that everything works as expected
    ```
    


## Troubleshooting

The main pixi tasks are related to CI.  Github actions runs the pixi task "ci".  The CI is mostly likey to fail from a lockfile mismatch.  Use `pixi run fix` to fix any lockfile related problems. 

## vscode tasks

There are two core tasks.  

1. set \<cfg\> from active file

    This sets \<cfg\> to the currently opened file in the editor

2. run \<cfg\>

    This runs python with the file set in \<cfg\>

