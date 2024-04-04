# deps_rocker

This is a [rocker](https://github.com/tfoote/rocker) extension for automating dependency installation.


The extension will recursivly search for deps.yaml files and run the install commands in several layers

Layer order:
- script_tools
- apt_tools
- pip_tools

- script_base
- apt_base
- pip_base

- script
- apt
- pip
- script_post

example deps.yaml

cuda
nvidia
apt_dev all apt dev dependencies
pip_dev pip dev dependencies
apt_large
apt
pip_large
pip


```
apt_base: #base apt dependencies that rarely change. Usually dev dep
  - git
  - git-lfs
  - python3-pip

pip_base: #base pip dependencies that rarely change
  - flit
  - pip

apt_large

apt: #project apt dependencies that may change on a more regular basis
  - nano
  - vim

pip: #project pip dependencies that may change on a more regular basis
 - pyyaml

```

run with:

```
rocker deps-dependencies ubuntu:22.04
```

## limitations/TODO

This has only been tested on the ubuntu base image. It assumes you have access to apt-get.

all the pip tags must have an entry for the moment.  Will improve the dockerfile logic to allow empty pip layers


