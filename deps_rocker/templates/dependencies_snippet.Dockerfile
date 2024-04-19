@# DEFINE EMPY MACROS FOR GENERATING DOCKERFILE

@# DEFINE EMPY FUNCTION FOR RUNNING SCRIPTS
@[def define_script(filename)]@
COPY @filename /@filename
RUN chmod +x /@filename; /@filename
@[end def]@

@# DEFINE EMPY FUNCTION FOR INSTALLING APT DEPS
@[def define_apt_deps(filename)]@
COPY @filename /@filename
RUN apt-get update \ 
 && apt-get install -y --no-install-recommends $(cat /@filename) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*
@[end def]@

@# DEFINE EMPY FUNCTION FOR PIP INSTALLING
@[def define_pip_install(filename)]@
COPY @filename /@filename
RUN pip3 install -U $(cat /@filename)
@[end def]@

@# END OF EMPY MACROS


#SET UP ENVIRONMENT VARIABLES
@[for x in env_vars]@
ENV @x
@[end for]@

#INSTALL DEVELOPMENT TOOLS
@define_script("scripts_tools.sh")
@define_apt_deps("apt_tools.deps")
@define_pip_install("pip_tools.deps")

#INSTALL EXPENSIVE BASE DEPENDENCIES
@define_script("scripts_base.sh")
@define_apt_deps("apt_base.deps")
@define_pip_install("pip_base.deps")

#INSTALL DEVELOPMENT DEPENDENCIES
@define_script("scripts.sh")
@define_apt_deps("apt.deps")
@define_pip_install("pip.deps")

#POST SETUP
@define_script("scripts_post.sh")