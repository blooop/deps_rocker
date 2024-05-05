# ADD APT REPOSITORIES:: @layer_name
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

@[for x in data_list]@
RUN add-apt-repository @x
@[end for]@