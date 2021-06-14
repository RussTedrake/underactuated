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

There was a time when I used some ajax to include source code. If you see
missing source code, you'll need to run a local webserver for the code includes
to work. I used the instructions at
https://websitebeaver.com/set-up-localhost-on-macos-high-sierra-apache-mysql-and-php-7-with-sslhttps
and just pointed by root doc directory directly at my underactuated checkout.


To run the linters and unit tests
---------------------------------

```
bazel test //...
```

If you would like to `bazel` to use a local installation of drake, you can set
the `DRAKE_INSTALL_DIR` environment variable. Otherwise it will look in
`/opt/drake`.

To get experimental Drake binaries
----------------------------------

Please see
the [relevant Drake documentation](https://drake.mit.edu/jenkins#building-binary-packages-on-demand)
.


Setting up PyCharm
------------------

In `Settings > Python Interpreter`, I set up a new "system interpreter". Then in
the Python Interpreter window, in the dropdown, use "show all" to see the
interpreters, then use "show paths for the selected interpreter" to add the path
to your drake installation (e.g. `/opt/drake/lib/python3.6/site-packages`).

PyCharm will eat up all of your memory trying to index everything, if you don't
also exclude the irrelevant directories from indexing (e.g. `.binder`, `data`,
`bazel-*`, `htmlbook/MathJax`). Do this by right-clicking on the directory in
the project view and selecting `Mark directory as > Excluded`.

I use selection and then Ctrl+alt+P for manual wrapping. I updated the HTML
style to use 2 for tabs/indent/continuation, and added `p`
to `Other > Do not indent children of`. I removed `h1` from the "indent before",
and unchecked "keep line breaks in text".

Additional information for Docker Hub users
-------------------------------------------

### Supported tags and respective `Dockerfile` links

- [`latest`](https://github.com/RussTedrake/underactuated/blob/master/scripts/docker/Dockerfile)

### Quick reference

* **Where to file issues**:
  [GitHub Issues](https://github.com/RussTedrake/underactuated/issues)

- **Maintained by**:
  [Russ Tedrake](https://github.com/RussTedrake)

- **Source of this description**:
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
