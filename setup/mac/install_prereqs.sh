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

# Pass the --retry 4 argument when invoking curl(1).
export HOMEBREW_CURL_RETRIES=4

# Do not send brew(1) usage analytics to Google Analytics.
export HOMEBREW_NO_ANALYTICS=1

# Do not automatically update before running various brew(1) subcommands.
export HOMEBREW_NO_AUTO_UPDATE=1

# Forbid redirects from secure HTTPS to insecure HTTP.
export HOMEBREW_NO_INSECURE_REDIRECT=1

# Never automatically cleanup installed, upgraded, and/or reinstalled formulae.
export HOMEBREW_NO_INSTALL_CLEANUP=1

# brew update uses git(1), so HOMEBREW_CURL_RETRIES does not take effect.
brew update || (sleep 30; brew update)
brew bundle --file="$(dirname ${(%):-%x})/Brewfile"
