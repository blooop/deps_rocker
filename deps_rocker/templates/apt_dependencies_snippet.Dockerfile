COPY apt_base.deps apt_base.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt_base.deps) \
 && apt-get clean 

COPY apt.deps apt.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt.deps) \
 && apt-get clean 

