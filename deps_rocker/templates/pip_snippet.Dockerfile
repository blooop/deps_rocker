@# INSTALLING PIP DEPS

COPY @filename /@filename
RUN pip3 install -U $(cat /@filename)