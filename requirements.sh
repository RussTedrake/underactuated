#!/bin/bash

# Runs pip-compile on the current platform and generates the shared
# requirements.txt
# pip-compile can be obtained via `pip3 install pip-tools`

if [[ "$OSTYPE" == "darwin"* ]]; then
    pip-compile mac-requirements.in
else        
    pip-compile ubuntu-requirements.in
fi

grep == mac-requirements.txt | sed -e 's/#.*$/ ; sys_platform == "darwin"/' > requirements.txt
grep == ubuntu-requirements.txt | sed -e 's/#.*$/; sys_platform == "linux"/' >> requirements.txt
