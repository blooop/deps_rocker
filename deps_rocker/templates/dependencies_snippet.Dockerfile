#INSTALL DEVELOPMENT TOOLS
COPY scripts_tools.sh /scripts_tools.sh
RUN chmod +x /scripts_tools.sh; /scripts_tools.sh

COPY apt_tools.deps /apt_tools.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt_tools.deps) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY pip_tools.deps /pip_tools.deps
RUN pip3 install -U $(cat /pip_tools.deps)

#INSTALL EXPENSIVE BASE DEPENDENCIES
COPY scripts_base.sh /scripts_base.sh
RUN chmod +x /scripts_base.sh; /scripts_base.sh

COPY apt_base.deps /apt_base.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt_base.deps) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY pip_base.deps /pip_base.deps
RUN pip3 install -U $(cat /pip_base.deps)

#INSTALL DEVELOPMENT DEPENDENCIES
COPY scripts.sh /scripts.sh
RUN chmod +x /scripts.sh; /scripts.sh

COPY apt.deps /apt.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt.deps) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY pip.deps /pip.deps
RUN pip3 install -U $(cat /pip.deps)

#POST SETUP
COPY scripts_post.sh /scripts_post.sh
RUN chmod +x /scripts_post.sh; /scripts_post.sh