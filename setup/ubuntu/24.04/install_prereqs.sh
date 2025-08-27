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

if [[ "$(lsb_release -cs)" != 'noble' ]]; then
  echo 'ERROR: This script requires Ubuntu 24.04 (Noble)' >&2
  exit 2
fi

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends ca-certificates gnupg

apt-get update -qq || (sleep 15; apt-get update -qq)

# Keep this up to date with Drake's
# setup/ubuntu/binary_distribution/packages-noble.txt, except that we choose to
# not install most of the system `python3-*` packages. The second batch are new
# requirements from this repo.
apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends $(cat <<EOF
build-essential
default-jre
jupyter-notebook
libblas-dev
libegl1
libeigen3-dev
libgfortran5
libglib2.0-0
libglx0
libgomp1
libjchart2d-java
liblapack3
libmumps-seq-5.6
libopengl0
libquadmath0
libpython3.12
libspdlog-dev
libx11-6
ocl-icd-libopencl1
python3
zlib1g

graphviz
jupyter-nbconvert
locales
python3-pip
tidy
wget
unzip
EOF
)


locale-gen en_US.UTF-8

# Note: I've purged widgets for now...
#jupyter nbextension enable --system --py widgetsnbextension

if [[ -z "${LANG:-}" && -z "${LC_ALL:-}" ]]; then
  echo 'WARNING: LANG and LC_ALL environment variables are NOT set. Please export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8.' >&2
fi

# This should match drake/setup/ubuntu/source_distribution/install_bazelisk.sh
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
  wget -O "${tmpdeb}" "${url}"
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

# If bazel.deb is already installed, we'll need to remove it first because
# the Debian package of bazelisk will take over the `/usr/bin/bazel` path.
apt-get remove bazel || true

# Install bazelisk.
if [[ $(arch) = "aarch64" ]]; then
  dpkg_install_from_wget \
    bazelisk 1.25.0 \
    https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-arm64.deb \
    6370ee55d7bb45b3511b7a1c1c93c565a5f5afcd24555820231c9c48beac95f3
else
  dpkg_install_from_wget \
    bazelisk 1.25.0 \
    https://github.com/bazelbuild/bazelisk/releases/download/v1.25.0/bazelisk-amd64.deb \
    f16dc348190990eb2e8950e773bc91dcdc7632517e5b63bdc4dd58f90062920c
fi
