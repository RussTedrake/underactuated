#!/bin/bash

# Copyright 2017 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

set -euxo pipefail

if [[ "${EUID:-}" -ne 0 ]]; then
  echo 'ERROR: This script must be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported. Please remove the Anaconda bin directory from the PATH.' >&2
fi

apt-get update -qq || (sleep 15; apt-get update -qq)

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends lsb-release

if [[ "$(lsb_release -cs)" != 'bionic' ]]; then
  echo 'ERROR: This script requires Ubuntu 18.04 (Bionic)' >&2
  exit 2
fi

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends ca-certificates gnupg

apt-get update -qq || (sleep 15; apt-get update -qq)

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends $(cat <<EOF
jupyter
jupyter-nbconvert
jupyter-notebook
locales
python3
python3-ipywidgets
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
  bazel 5.0.0 \
  https://releases.bazel.build/5.0.0/release/bazel_5.0.0-linux-x86_64.deb \
  e3361645ccd731abc424bb3a322d8e6f513b7258f5ca11ff04d6067aff5d09b1
