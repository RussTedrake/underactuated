#!/bin/bash

set -e

python3 htmlbook/publish_to_deepnote.py $(cat Deepnote_docker_sha.txt)
echo "Remember to log on to deepnote and build the dockerfile in any one of the notebooks"
echo "https://deepnote.com/workspace/$(cat Deepnote_workspace.txt)"