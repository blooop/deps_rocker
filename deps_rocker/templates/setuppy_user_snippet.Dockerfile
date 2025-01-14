
# RUN pip install vcstool


# RUN \
@[for x in data_list]@
RUN cd @x; pip3 install -e . \
@[end for]@
