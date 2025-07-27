# INSTALLING APT DEPS: @layer_name
# Install whatever your extension needs
RUN apt-get update && apt-get install -y --no-install-recommends \	RUN apt-get update && apt-get install -y --no-install-recommends \
    @[for x in data_list]@	    my-package \
    @x \	
    @[end for]@	
    && apt-get clean && rm -rf /var/lib/apt/lists/*