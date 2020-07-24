<!-- This file automatically populates the overview page on Docker Hub. -->

Underactuated Robotics
======================

*Algorithms for Walking, Running, Swimming, Flying, and Manipulation*

<http://underactuated.mit.edu/>

[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/RussTedrake/underactuated/master)
![CI](https://github.com/RussTedrake/underactuated/workflows/CI/badge.svg)
[![Docker Build](https://img.shields.io/docker/cloud/build/russtedrake/underactuated?logo=docker)](https://hub.docker.com/r/russtedrake/underactuated)

Follow the installation instructions in http://underactuated.mit.edu/drake.html

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


*Boilerplate on hypothes.is etiquette*:  For typos and small corrections, please remove your comment once it has been addressed.


To run the linters and unit tests
---------------------------------

```
bazel test //...
```

If you would like to `bazel` to use a local installation of drake, you can set
the `DRAKE_INSTALL_DIR` environment variable.  Otherwise it will look in
`/opt/drake`.

To get experimental Drake binaries
----------------------------------

Please see the [relevant Drake documentation](https://drake.mit.edu/jenkins#building-binary-packages-on-demand).

Additional information for Docker Hub users
-------------------------------------------

### Supported tags and respective `Dockerfile` links

-	[`latest`](https://github.com/RussTedrake/underactuated/blob/master/scripts/docker/Dockerfile)

### Quick reference

* **Where to get help**:
  [Stack Overflow](https://stackoverflow.com/questions/tagged/underactuated)

* **Where to file issues**:
  [GitHub Issues](https://github.com/RussTedrake/underactuated/issues)

-	**Maintained by**:
	[Russ Tedrake](https://github.com/RussTedrake)

-	**Supported architectures**:
  `amd64`

-	**Source of this description**:
	[`README.md`](https://github.com/RussTedrake/underactuated/blob/master/README.md)
	([history](https://github.com/RussTedrake/underactuated/commits/master/README.md))

### License

View [license information](https://github.com/RussTedrake/underactuated/blob/master/LICENSE.TXT)
for the software contained in this image.

As with all Docker images, these likely also contain other software which may be
under other licenses (such as Bash, etc from the base distribution, along with
any direct or indirect dependencies of the primary software being contained).

As for any pre-built image usage, it is the image user's responsibility to
ensure that any use of this image complies with any relevant licenses for all
software contained within.
