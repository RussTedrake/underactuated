# -*- mode: dockerfile -*-
# vi: set ft=dockerfile :

# Copyright (c) 2020, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
COPY underactuated/ $HOME/underactuated
COPY examples/ $HOME/examples
RUN chown -R $NB_UID:$NB_GID $HOME/underactuated \
  && find $HOME/underactuated \( -name BUILD.bazel -or -name CMakeLists.txt \) -delete
RUN chown -R $NB_UID:$NB_GID $HOME/examples \
  && find $HOME/examples \( -name BUILD.bazel -or -name CMakeLists.txt \) -delete
USER $NB_USER
ENTRYPOINT ["/opt/underactuated/bin/entrypoint"]
CMD ["jupyter", "notebook", "--ip", "0.0.0.0"]
