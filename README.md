Underactuated Robotics
======================

*Algorithms for Walking, Running, Swimming, Flying, and Manipulation*

<http://underactuated.mit.edu/>

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RussTedrake/underactuated/master)
![CI](https://github.com/RussTedrake/underactuated/workflows/CI/badge.svg)

Follow the installation instructions in 
http://underactuated.mit.edu/drake.html


To view the text locally
------------------------

Make sure to initialize the submodules:

```
git submodule update --init --recursive
```

The textbook should then be viewable by opening `underactuated.html` in your
browser.

You'll need to run a local webserver for the code includes (via ajax) to work. I
used the instructions at 
https://websitebeaver.com/set-up-localhost-on-macos-high-sierra-apache-mysql-and-php-7-with-sslhttps
and just pointed by root doc directory directly at my underactuated checkout.

To run the linters and unit tests
---------------------------------

```
bazel test //...
```

To get experimental Drake binaries
----------------------------------

Please see the [relevant Drake documentation](https://drake.mit.edu/jenkins#building-binary-packages-on-demand).
