# deps_rocker

This is a [rocker](https://github.com/tfoote/rocker) extension for automating dependency installation.


The extension will recursivly search for deps.yaml files and run the install commands in several layers

Layer order:

- apt_base
- pip_base
- apt
- pip

example deps.yaml

```
apt_base: #base apt dependencies that rarely change
  - git
  - git-lfs
  - python3-pip

pip_base: #base pip dependencies that rarely change
  - flit
  - pip

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


