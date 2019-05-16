Underactuated Robotics
======================

*Algorithms for Walking, Running, Swimming, Flying, and Manipulation*

<http://underactuated.mit.edu/>

Follow the installation instructions in 
http://underactuated.mit.edu/underactuated.html?ch=drake


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


To run the unit tests
---------------------

```
$ mkdir build && cd build
$ cmake -Ddrake_DIR=PATH_TO_DRAKE/lib/cmake/drake ..
$ make
$ ctest .
```



To get experimental drake binaries
-----------------------------------

As described at http://github.com/RobotLocomotion/drake/issues/7926, use

On your drake PR, use
```
@drake-jenkins-bot linux-xenial-unprovisioned-gcc-bazel-experimental-snopt-packaging please
@drake-jenkins-bot mac-mojave-unprovisioned-clang-bazel-experimental-snopt-packaging please
```
Then examine the last lines of the console output from those builds for the 
binary urls.  
