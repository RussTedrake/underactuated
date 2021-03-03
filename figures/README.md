
To convert movies into ogg/ogv
```
brew remove ffmpeg
brew install --with-theora --with-libvorbis ffmpeg
ffmpeg -i test.mov -vcodec libtheora -acodec libvorbis test.ogv
```

To convert xfig to svg, seems better today (2021-03-02, on ubuntu 18.04) to use xfig 
to export to pdf, then pdf2svg.  (otherwise the fonts didn't come out right.)