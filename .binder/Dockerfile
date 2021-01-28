# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# Copyright 2021 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

FROM russtedrake/underactuated:latest
ARG NB_USER=jovyan
ARG NB_UID=1000
ARG NB_GID=100
RUN useradd -d /home/${NB_USER} -G ${NB_GID} -mU -s /bin/bash ${NB_USER}
ENV HOME /home/${NB_USER}
ENV PYTHONPATH ${HOME}:${PYTHONPATH}
ENV USER ${NB_USER}
WORKDIR ${HOME}
COPY .binder/entrypoint /opt/underactuated/bin/
COPY . ${HOME}
RUN chown -R ${NB_UID}:${NB_GID} ${HOME} \
  && find ${HOME} \( \
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
USER ${NB_USER}
ENTRYPOINT ["/opt/underactuated/bin/entrypoint"]
CMD ["jupyter", "notebook", "--ip", "0.0.0.0"]
