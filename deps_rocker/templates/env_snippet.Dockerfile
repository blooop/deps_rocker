# SET UP ENVIRONMENT VARIABLES: @layer_name
@[for x in data_list]@
ENV @x
@[end for]@