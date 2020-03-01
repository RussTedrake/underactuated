# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

# cd $(git rev-parse --show-toplevel)
# docker build -f scripts/setup/ubuntu/18.04/Dockerfile -t robotlocomotion/underactuated:latest .
#
# docker run -i -P -t robotlocomotion/underactuated:latest
#
# docker login
# docker push robotlocomotion/underactuated:latest

FROM robotlocomotion/drake:latest
ARG DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
EXPOSE 7000/tcp
EXPOSE 8888/tcp
COPY scripts/docker/pip.conf /root/.config/pip/pip.conf
COPY scripts/setup/ubuntu/18.04/install_prereqs /tmp/
RUN /tmp/install_prereqs \
 && rm -rf /opt/drake/share/sdformat \
 && rm -rf /root/.cache \
 && rm -rf /root/.config \
 && rm -f /tmp/install_prereqs \
 && rm -f /var/cache/debconf/*-old \
 && rm -rf /var/lib/apt/lists/* \
 && rm -f /var/log/*.log \
 && rm -f /var/log/apt/*
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8
ENV LC_ALL en_US.UTF-8
