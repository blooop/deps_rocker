# INSTALLING APT DEPS: @layer_name
RUN apt-get update && apt-get install -y --no-install-recommends \
    @[for x in data_list]@
    @x \
    @[end for]@
    && apt-get clean && rm -rf /var/lib/apt/lists/*