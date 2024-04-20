@# DEFINE EMPY MACROS FOR GENERATING DOCKERFILE

@# DEFINE EMPY FUNCTION FOR RUNNING SCRIPTS
@[def define_script(filename,file_exists)]@
@[if file_exists]@
COPY @filename /@filename
RUN chmod +x /@filename; /@filename
@[end if]@
@[end def]@

@# DEFINE EMPY FUNCTION FOR INSTALLING APT DEPS
@[def define_apt_deps(filename,file_exists)]@
@[if file_exists]@
COPY @filename /@filename
RUN apt-get update \ 
 && apt-get install -y --no-install-recommends $(cat /@filename) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*
@[end if]@
@[end def]@

@# DEFINE EMPY FUNCTION FOR PIP INSTALLING
@[def define_pip_install(filename,file_exists)]@
@[if file_exists]@
COPY @filename /@filename
RUN pip3 install -U $(cat /@filename)
@[end if]@
@[end def]@

@# END OF EMPY MACROS


#SET UP ENVIRONMENT VARIABLES
@[for x in env_vars]@
ENV @x
@[end for]@

#INSTALL DEVELOPMENT TOOLS
@define_script("scripts_tools",scripts_tools)
@define_apt_deps("apt_tools",apt_tools)
@define_pip_install("pip_tools",pip_tools)

#INSTALL EXPENSIVE BASE DEPENDENCIES
@define_script("scripts_base",scripts_base)
@define_apt_deps("apt_base",apt_base)
@define_pip_install("pip_base",pip_base)

#INSTALL DEVELOPMENT DEPENDENCIES
@define_script("scripts",scripts)
@define_apt_deps("apt",apt)
@define_pip_install("pip",pip)

#INSTALL FROM PYPROJECT.TOML
@define_pip_install("pyproject_toml",pyproject_toml)

#POST SETUP
@define_script("scripts_post",scripts_post)