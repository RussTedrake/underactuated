#!/bin/bash

set -e

docker pull robotlocomotion/drake:focal
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:latest .
docker push russtedrake/underactuated:latest
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:$(git rev-parse --short HEAD) .
docker push russtedrake/underactuated:$(git rev-parse --short HEAD)
python3 htmlbook/publish_to_deepnote.py $(git rev-parse --short HEAD)
echo "Remember to log on to deepnote and build the dockerfile in any one of the notebooks"
echo "https://deepnote.com/workspace/$(cat Deepnote_workspace.txt)"