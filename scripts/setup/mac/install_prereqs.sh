#!/bin/zsh

# Copyright 2017 Massachusetts Institute of Technology.
# Licensed under the BSD 3-Clause License. See LICENSE.TXT for details.

set -euxo pipefail

if [[ "${EUID:-}" -eq 0 ]]; then
  echo 'ERROR: This script must NOT be run as root' >&2
  exit 1
fi

if command -v conda &>/dev/null; then
  echo 'WARNING: Anaconda is NOT supported. Please remove the Anaconda bin directory from the PATH.' >&2
fi

if ! command -v brew &>/dev/null; then
  bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

export HOMEBREW_CURL_RETRIES=4

brew update
if [[ -e /usr/local/bin/bazelisk ]]; then brew rm bazelisk; fi;
brew bundle --file="$(dirname ${(%):-%x})/Brewfile" --no-lock
