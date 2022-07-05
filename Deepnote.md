# For developers

See manipulation/Deepnote.md for most details.  Here are the relevant commands for updating the underactuated repos:
```
docker pull robotlocomotion/drake:focal
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:latest .
docker push russtedrake/underactuated:latest
docker build -f setup/docker/Dockerfile -t russtedrake/underactuated:$(git rev-parse --short HEAD) .
docker push russtedrake/underactuated:$(git rev-parse --short HEAD)
python3 htmlbook/publish_to_deepnote.py $(git rev-parse --short HEAD)
echo "Remember to log on to deepnote and build the dockerfile in any one of the notebooks"
```
