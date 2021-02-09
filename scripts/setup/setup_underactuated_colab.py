import importlib
import os
import subprocess
import sys
from urllib.request import urlretrieve


def setup_drake(*, version, build):
    urlretrieve(
        f"https://drake-packages.csail.mit.edu/drake/{build}/drake-{version}/setup_drake_colab.py",
        "setup_drake_colab.py")
    import setup_drake_colab
    setup_drake_colab.setup_drake(version=version, build=build)


def setup_underactuated(*, underactuated_sha, drake_version, drake_build):
    setup_drake(version=drake_version, build=drake_build)

    path = "/opt/underactuated"

    def run(cmd, **kwargs):
        cp = subprocess.run(cmd,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            universal_newlines=True, **kwargs)
        if cp.stderr:
            print(cp.stderr)
        assert cp.returncode == 0, cp

    # Clone the repo (if necessary).
    if not os.path.isdir(path):
        run([
            'git', 'clone', 'https://github.com/RussTedrake/underactuated.git',
            path
        ])

    # Checkout the sha.
    run(['git', 'checkout', '--detach', underactuated_sha], cwd=path)

    # Run install_prereqs.sh
    run([f"{path}/scripts/setup/ubuntu/18.04/install_prereqs.sh"])

    # Run pip install
    if os.path.isfile("/opt/underactuated/colab-requirements.txt"):
        run([
            "pip3", "install", "--requirement",
            "/opt/underactuated/colab-requirements.txt"
        ])

    # Install colab specific requirements
    run(["apt", "install", "xvfb"])

    # Set the path (if necessary).
    spec = importlib.util.find_spec('underactuated')
    if spec is None:
        sys.path.append(path)
        spec = importlib.util.find_spec('underactuated')

    # Confirm that we now have underactuated on the path.
    assert spec is not None, (
        "Installation failed.  find_spec('underactuated') returned None.")
    assert path in spec.origin, (
        "Installation failed.  find_spec is locating underactuated, but not "
        "in the expected path.")
