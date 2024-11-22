# IMPORTING VCSTOOL repos: @layer_name

RUN pip install vcstool

@[for x in data_list]@
RUN vcs import  < @x --recursive
@[end for]@