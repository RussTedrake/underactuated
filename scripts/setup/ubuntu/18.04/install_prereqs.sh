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

APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 apt-key adv \
  -q --fetch-keys https://bazel.build/bazel-release.pub.gpg

echo 'deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8' \
  > /etc/apt/sources.list.d/bazel.list

apt-get update -qq || (sleep 15; apt-get update -qq)

apt-get install -o APT::Acquire::Retries=4 -o Dpkg::Use-Pty=0 -qy \
  --no-install-recommends $(cat <<EOF
bazel
jupyter
jupyter-nbconvert
jupyter-notebook
locales
python3
python3-bs4
python3-future
python3-ipywidgets
python3-matplotlib
python3-numpy
python3-pip
python3-requests
python3-scipy
python3-setuptools
python3-wheel
python3-widgetsnbextension
tidy
wget
EOF
)

locale-gen en_US.UTF-8

jupyter nbextension enable --system --py widgetsnbextension

if [[ -z "${LANG:-}" && -z "${LC_ALL:-}" ]]; then
  echo 'WARNING: LANG and LC_ALL environment variables are NOT set. Please export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8.' >&2
fi
