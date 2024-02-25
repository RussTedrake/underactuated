#!/bin/bash

# Copyright (c) 2019, Massachusetts Institute of Technology.
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

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported. Please remove the Anaconda bin directory from the PATH.' >&2
fi

apt-get update -qq || (sleep 15; apt-get update -qq)

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends lsb-release

if [[ "$(lsb_release -cs)" != 'jammy' ]]; then
  echo 'ERROR: This script requires Ubuntu 22.04 (Jammy)' >&2
  exit 2
fi

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends ca-certificates gnupg

apt-get update -qq || (sleep 15; apt-get update -qq)

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends $(cat <<EOF
graphviz
jupyter
jupyter-nbconvert
jupyter-notebook
libglib2.0-0
libsm6
libx11-6
locales
python3
python3-pip
python3-widgetsnbextension
tidy
wget
g++
unzip
zlib1g-dev
EOF
)

locale-gen en_US.UTF-8

jupyter nbextension enable --system --py widgetsnbextension

if [[ -z "${LANG:-}" && -z "${LC_ALL:-}" ]]; then
  echo 'WARNING: LANG and LC_ALL environment variables are NOT set. Please export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8.' >&2
fi


dpkg_install_from_wget() {
  package="$1"
  version="$2"
  url="$3"
  checksum="$4"

  # Skip the install if we're already at the exact version.
  installed=$(dpkg-query --showformat='${Version}\n' --show "${package}" 2>/dev/null || true)
  if [[ "${installed}" == "${version}" ]]; then
    echo "${package} is already at the desired version ${version}"
    return
  fi

  # If installing our desired version would be a downgrade, ask the user first.
  if dpkg --compare-versions "${installed}" gt "${version}"; then
    echo "This system has ${package} version ${installed} installed."
    echo "Drake suggests downgrading to version ${version}, our supported version."
    read -r -p 'Do you want to downgrade? [Y/n] ' reply
    if [[ ! "${reply}" =~ ^([yY][eE][sS]|[yY])*$ ]]; then
      echo "Skipping ${package} ${version} installation."
      return
    fi
  fi

  # Download and verify.
  tmpdeb="/tmp/${package}_${version}-amd64.deb"
  wget -q -O "${tmpdeb}" "${url}"
  if echo "${checksum} ${tmpdeb}" | sha256sum -c -; then
    echo  # Blank line between checkout output and dpkg output.
  else
    echo "ERROR: The ${package} deb does NOT have the expected SHA256. Not installing." >&2
    exit 2
  fi

  # Install.
  dpkg -i "${tmpdeb}"
  rm "${tmpdeb}"
}

dpkg_install_from_wget \
  bazel 6.1.1 \
  https://releases.bazel.build/6.1.1/release/bazel_6.1.1-linux-x86_64.deb \
  a90246165f0972629506132975a7c5d5aecd42453e03e0f88e175a33601cdf70
