# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# Copyright 2021 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

# cd $(git rev-parse --show-toplevel)
# docker build -f scripts/docker/Dockerfile \
#   -t robotlocomotion/underactuated:latest .
#
# docker run -i -p 7000:7000 -p 8888:8888 -t -v $(pwd):/root/underactuated \
#   -w /root/underactuated robotlocomotion/underactuated:latest
#
# docker login
# docker push robotlocomotion/underactuated:latest

FROM robotlocomotion/drake:latest
ARG DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
EXPOSE 7000/tcp
EXPOSE 8888/tcp
LABEL org.opencontainers.image.authors="Russ Tedrake"
LABEL org.opencontainers.image.description="Algorithms for walking, running, swimming, flying, and manipulation"
LABEL org.opencontainers.image.licenses="BSD-3-Clause"
LABEL org.opencontainers.image.source="https://github.com/RussTedrake/underactuated.git"
LABEL org.opencontainers.image.title="Underactuated Robotics"
LABEL org.opencontainers.image.url="http://underactuated.mit.edu/"
LABEL org.opencontainers.image.vendor="Massachusetts Institute of Technology"
COPY scripts/docker/pip.conf /root/.config/pip/pip.conf
COPY scripts/setup/ubuntu/18.04/install_prereqs.sh /tmp/
RUN /tmp/install_prereqs.sh \
 && rm -rf /root/.cache \
 && rm -rf /root/.config \
 && rm -f /tmp/install_prereqs.sh \
 && rm -f /var/cache/debconf/*-old \
 && rm -rf /var/lib/apt/lists/* \
 && rm -f /var/log/*.log \
 && rm -f /var/log/apt/*
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8
