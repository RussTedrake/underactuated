Underactuated Robotics
======================

*Algorithms for Walking, Running, Swimming, Flying, and Manipulation*

<http://underactuated.mit.edu/>

Build the examples
------------------

```
$ mkdir build && cd build
$ cmake -Ddrake_DIR=PATH_TO_DRAKE/lib/cmake/drake ..
$ make
```

Also make sure to initialize the submodules:

```
git submodule init
git submodule update
```
The textbook should then be viewable by opening `underactuated.html` in your browser.

You'll need to run a local webserver for the code includes (via ajax) to work. I used https://websitebeaver.com/set-up-localhost-on-macos-high-sierra-apache-mysql-and-php-7-with-sslhttps and just pointed by root doc directory directly at my underactuated checkout. 

Notes
-----

Embedding mp4 w/ flash fallback:

<http://css-tricks.com/snippets/html/video-for-everybody-html5-video-with-flash-fallback/>


HTML linter
-----------

mac:  brew install tidy-html5
linux: sudo apt-get install tidy

tidy -config .tidy.config -f errs.txt underactuated.html

