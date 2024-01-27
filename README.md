# deps_rocker

This is a [rocker](https://github.com/tfoote/rocker) extension for automating dependency installation.

run with:

```
rocker vs-dependencies ubuntu:22.04
```

The extension will recursivly search for dependencies.yaml files and run the install commands in several layers

Layer order:

- apt_base
- pip_base
- apt
- pip


```
apt_base: #base image dependencies that rarely change
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

limitations/TODO

all the pip tags must have an entry for the moment.  Will improve the dockerfile logic to allow empty pip layers

