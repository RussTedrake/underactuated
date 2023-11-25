## To Run the Unit Tests

Make sure that you have done a recursive checkout in this repository, or have run

```bash
git submodule update --init --recursive
```
Then run
```bash
bazel test //...
```

## To install poetry

Install poetry using the [official installer](https://python-poetry.org/docs/#installing-with-the-official-installer); not brew nor apt.
Install the poetry export plugin:
```
pip3 install poetry-plugin-export
```

## To update poetry

```
./htmlbook/PoetryExport.sh
```
One may want to also run
```
poetry install
```

## Additional setup on macos arm64

Currently, after pip installing nbconvert on apple m1, the `jupyter --paths`
command still doesn't include `/opt/homebrew/share/jupyter`.  As a result,
nbconvert fails to find the python template.  Hopefully this will be fixed, but
for now a work-around is:

```
% cd ~/Library/Jupyter/nbconvert/
% mv templates templates_bk
% ln -s /opt/homebrew/share/jupyter/nbconvert/templates
```

## To update the pip wheels

Update the version number in `pyproject.toml`, then from the root directory, run:
```
rm -rf dist/*
poetry publish --build
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

## Formatting

I use `black` + `isort` + `autoflake` as pre-commit hooks.  `black` and `isort`
support jupyter files directly, but `autoflake` does not, so I run `cleanipynb`
(which runs `autopep8`), and then rerun `black`.  

Run `pre-commit install` to install the pre-commit hooks.

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

