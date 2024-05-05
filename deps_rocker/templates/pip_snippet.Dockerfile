# INSTALLING PIP DEPS: @layer_name
RUN pip3 install -U \
@[for x in data_list]@
    @x \
@[end for]@
    && echo "pip"