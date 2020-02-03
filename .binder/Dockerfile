# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# Copyright 2020 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

FROM robotlocomotion/drake:latest
ARG NB_USER=jovyan
ARG NB_UID=1000
ARG NB_GID=100
EXPOSE 8888/tcp
COPY scripts/setup/ubuntu/18.04/install_prereqs /tmp/
RUN export DEBIAN_FRONTEND=noninteractive \
  && /tmp/install_prereqs > /dev/null 2>&1 \
  && rm -rf /tmp/install_prereqs /var/lib/apt/lists/* \
  && useradd -d /home/$NB_USER -G $NB_GID -mU -s /bin/bash $NB_USER
ENV HOME=/home/$NB_USER \
  LANG=en_US.UTF-8 \
  LANGUAGE=en_US.UTF-8 \
  LC_ALL=en_US.UTF-8 \
  PYTHONPATH=/home/$NB_USER:$PYTHONPATH \
  SHELL=/bin/bash \
  USER=$NB_USER
WORKDIR $HOME
COPY .binder/entrypoint /opt/underactuated/bin/
COPY . $HOME
RUN chown -R $NB_UID:$NB_GID $HOME \
  && find $HOME \( \
      -name .editorconfig -or \
      -name .yapfignore -or \
      -name BUILD.bazel -or \
      -name CMakeLists.txt -or \
      -name README.md \
    \) -delete \
  && rm -rf \
    .bazelrc \
    .binder \
    .circleci \
    .github \
    drake_version.json \
    LICENSE.TXT \
    requirements.txt \
    scripts \
    setup.cfg \
    tools \
    WORKSPACE.bazel
USER $NB_USER
ENTRYPOINT ["/opt/underactuated/bin/entrypoint"]
CMD ["jupyter", "notebook", "--ip", "0.0.0.0"]
