#!/bin/bash

case $1 in
  ("homebrew")
    pip install beautifulsoup4 ;;
  ("macports")
    pip install beautifulsoup4 ;;
  ("ubuntu")
    apt-get install python-bs4 ;;
  (*)
    echo "Usage: ./install_prereqs.sh package_manager"
    echo "where package_manager is one of the following: "
    echo "  homebrew"
    echo "  macports"
    echo "  ubuntu"
    exit 1 ;;
esac
