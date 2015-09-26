#!/bin/bash

case $1 in
  ("homebrew")
    pip install beautifulsoup4 html5lib ;;
  ("macports")
    pip install beautifulsoup4 html5lib ;;
  ("ubuntu")
      apt-get install python-bs4 python-html5lib ;;
  ("cygwin")
      cygwin-setup -q -P python python-setuptools 
      /usr/bin/easy_install pip 
      pip install beautifulsoup4 html5lib ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    echo "  cygwin"
    exit 1 ;;
esac
