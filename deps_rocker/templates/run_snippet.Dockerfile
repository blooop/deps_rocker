# RUN COMMANDS: @layer_name
RUN \
@[for x in data_list]@
    @x \
@[end for]@
    && echo "end run"