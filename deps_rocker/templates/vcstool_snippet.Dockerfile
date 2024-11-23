# IMPORTING VCSTOOL repos
RUN apt-get update; apt-get install python3-pip -y

RUN pip3 install vcstool



COPY depend.repos /deps.repos


RUN pip install vcstool

@[for x in data_list]@
RUN vcs import  < @x --recursive
@[end for]@