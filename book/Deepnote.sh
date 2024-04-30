#!/bin/bash

set -e

python3 htmlbook/publish_to_deepnote.py $(cat Deepnote_docker_sha.txt)
