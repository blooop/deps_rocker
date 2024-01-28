COPY apt_base.deps apt.deps
RUN apt-get update \ 
 && apt-get install -y $(cat /apt.deps) \
 && apt-get clean 