#!/bin/bash

set -e

docker pull robotlocomotion/drake:jammy
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:latest .
docker push russtedrake/underactuated:latest
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:$(git rev-parse --short HEAD) .
docker push russtedrake/underactuated:$(git rev-parse --short HEAD)
git rev-parse --short HEAD > book/Deepnote_docker_sha.txt
echo "Remember to run Deepnote.sh to actually push to Deepnote"
echo "Remember to log on to deepnote and build the dockerfile in any one of the notebooks"
echo "https://deepnote.com/workspace/$(cat Deepnote_workspace.txt)"