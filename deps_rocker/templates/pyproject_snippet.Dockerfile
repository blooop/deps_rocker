# INSTALLING PYPROJECT DEPS: @layer_name
COPY @filename /@filename
RUN pip3 install -U $(cat /@filename)