
To convert movies into ogg/ogv
```
brew remove ffmpeg
brew install --with-theora --with-libvorbis ffmpeg
ffmpeg -i test.mov -vcodec libtheora -acodec libvorbis test.ogv
```
