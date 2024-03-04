# RUN apt-get update ; apt-get install \
#     apt-transport-https \
#     ca-certificates \
#     curl \
#     gnupg-agent \
#     software-properties-common -y


# RUN curl -fsSL



# COPY rocker-latest.list /etc/apt/sources.list.d/rocker-latest.list
COPY rocker-latest.list rocker-latest.list

RUN add-apt-repository $(cat rocker-latest.list)

# COPY scripts_base.sh scripts_base.sh

# RUN chmod +x /scripts_base.sh; /scripts_base.sh

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
