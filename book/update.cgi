#!/usr/bin/env python3

import os
import subprocess

try:
    # Do all work first
    git_output = subprocess.run(
        ["git", "fetch", "origin"], capture_output=True, text=True, check=True
    ).stdout

    git_output += subprocess.run(
        ["git", "reset", "--hard", "origin/master"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout

    git_output += subprocess.run(
        ["git", "submodule", "update", "--init", "--recursive"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout

    os.chdir("..")

    build_output = subprocess.run(
        [
            "/bin/bash",
            "-c",
            "source venv/bin/activate && "
            "poetry install --only docs && "
            "echo 'Contents of /tmp before:' && ls -la /tmp && "
            "sphinx-build -M html underactuated /tmp/underactuated_doc && "
            "echo 'Contents of /tmp after:' && ls -la /tmp && "
            "rm -rf book/python && "
            "cp -r /tmp/underactuated_doc/html book/python",
        ],
        capture_output=True,
        text=True,
        check=True,
    ).stdout

    # Now we can safely print headers and content
    print("Content-Type: text/html\n")
    print("<html><body>")
    print("<p>pulling repo...</p>")
    print(f"<pre>{git_output}</pre>")
    print("<p>done.</p>")
    print("<p>building documentation...</p>")
    print(f"<pre>{build_output}</pre>")
    print("<p>done.</p>")
    print("</body></html>")

except Exception as e:
    print("Content-Type: text/html\n")
    print("<html><body>")
    print(f"<p>Error: {str(e)}</p>")
    print("</body></html>")
