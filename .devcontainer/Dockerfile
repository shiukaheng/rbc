FROM ubuntu:20.04

ENTRYPOINT [ "/usr/bin/env" ]

# Copy in ./apt-get-wrapper.sh
COPY ./apt-get-wrapper.sh /usr/local/bin/apt-get-wrapper.sh
RUN chmod +x /usr/local/bin/apt-get-wrapper.sh

# Set password for root to be workspace
RUN echo 'root:workspace' | chpasswd

ENV DEV_ENV=1