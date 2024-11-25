# IMPORTING VCSTOOL repos
RUN apt-get update; apt-get install python3-pip -y

RUN pip3 install vcstool



# COPY depend.repos /deps.repos

#tmp