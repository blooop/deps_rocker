COPY scripts_base.sh scripts_base.sh

RUN chmod +x /scripts_base.sh; /scripts_base.sh

COPY apt_base.deps apt_base.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt_base.deps) \
 && apt-get clean 

COPY pip_base.deps pip_base.deps
RUN pip install -U $(cat /pip_base.deps)

# COPY scripts.deps scripts.sh

# RUN scripts.sh

COPY apt.deps apt.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt.deps) \
 && apt-get clean 

COPY pip.deps pip.deps
RUN pip install -U $(cat /pip.deps)
