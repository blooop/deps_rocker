RUN locale-gen en_US.UTF-8 && \
    echo "LANG=en_US.UTF-8" > /etc/locale.conf && \
    echo "en_US.UTF-8 UTF-8" >> /etc/locale.gen && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=en_US.UTF-8