@# INSTALLING APT DEPS

COPY @filename /@filename
RUN apt-get update \ 
 && apt-get install -y --no-install-recommends $(cat /@filename) \
 && apt-get clean && rm -rf /var/lib/apt/lists/*