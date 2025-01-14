
# RUN pip install vcstool

# #loops through all the *.repos files that were found and imports them with the same folder structure

# RUN \
@[for x in data_list]@
RUN cd @x; pip3 install -e . \
@[end for]@
