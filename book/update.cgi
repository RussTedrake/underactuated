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

    # Start the build process in background
    with open("/tmp/underactuated_build_docs.log", "w") as log_file:
        subprocess.Popen(
            [
                "/bin/bash",
                "-c",
                "source venv/bin/activate && "
                "poetry install --only docs && "
                "sphinx-build -M html underactuated /tmp/underactuated_doc && "
                "rm -rf book/python && "
                "cp -r /tmp/underactuated_doc/html book/python",
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    print("Content-Type: text/html\n")
    print("<html><body>")
    print("<p>pulling repo...</p>")
    print(f"<pre>{git_output}</pre>")
    print("<p>done.</p>")
    print("<p>Documentation build started in the background.</p>")
    print("<p>Check /tmp/underactuated_build_docs.log for progress.</p>")
    print("</body></html>")

except Exception as e:
    print("Content-Type: text/html\n")
    print("<html><body>")
    print(f"<p>Error: {str(e)}</p>")
    print("</body></html>")
