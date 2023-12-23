
To convert movies into ogg/ogv
```
brew remove ffmpeg
brew install --with-theora --with-libvorbis ffmpeg
ffmpeg -i test.mov -vcodec libtheora -acodec libvorbis test.ogv
```

To convert xfig to svg, seems better today (2021-03-02, on ubuntu 18.04) to use xfig 
to export to pdf, then pdf2svg.  (otherwise the fonts didn't come out right.)


Example matplotlib figure export
```
plt.savefig('data/cubic_polynomial_outer_approx.svg', transparent=True, pad_inches=0.0)
```

svg figures using omnigraffle, with latexit for equations via pdf.
export to pdf (w/ transparent background).  pdf2svg.
(direct to svg would presumably work if I didn't have the latex pdf equations)
