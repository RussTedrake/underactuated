Underactuated Robotics
======================

*Algorithms for Walking, Running, Swimming, Flying, and Manipulation*

<http://underactuated.mit.edu/>

Prerequisites
-------------

Debian / Ubuntu:
```
$ sudo apt-get install python-bs4 python-html5lib
```

macOS / OS X

```
$ pip install beautifulsoup4 html5lib
```

Build
-----

```
$ mkdir build && cd build
$ cmake ..
$ make
```

Also make sure to initialize the submodules:

```
git submodule init
git submodule update
```
The textbook should then be viewable by opening `index.html` in your browser. 

Notes
-----

Embedding mp4 w/ flash fallback:

<http://css-tricks.com/snippets/html/video-for-everybody-html5-video-with-flash-fallback/>
