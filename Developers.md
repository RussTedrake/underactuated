## To Run the Unit Tests

Make sure that you have done a recursive checkout in this repository, or have run

```bash
git submodule update --init --recursive
```
Then run
```bash
bazel test //...
```

If you would like to `bazel` to use a local installation of drake, you can set
the `DRAKE_INSTALL_DIR` environment variable. Otherwise it will look in
`/opt/drake`.

## To update the pip wheels

Update the version number in `setup.py`, then from the root directory, run:
```
python3 -m pip install --upgrade build twine
python3 -m build
python3 -m twine upload dist/*
``` 

## Tips for developers

These are things that I often add to my preamble of the notebook (ever since vs code broke my pythonpath importing)
```
%load_ext autoreload
%autoreload 2
import sys
sys.path.append('/home/russt/drake-install/lib/python3.6/site-packages')
sys.path.append('/home/russt/manipulation')
```

## Sorting imports in VS Code

First `pip install isort`.  Then in the command pallet, run `Python Refactor:Sort Imports`.

## Setting up PyCharm

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

## Ajax

There was a time when I used some ajax to include source code. If you are
viewing the notes on a local machine and are missing source code, you'll need
to run a local webserver for the code includes to work. I used the instructions
at
https://websitebeaver.com/set-up-localhost-on-macos-high-sierra-apache-mysql-and-php-7-with-sslhttps
and just pointed by root doc directory directly at my underactuated checkout.

