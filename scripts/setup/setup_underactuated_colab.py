import importlib
import os
import subprocess
import sys
from urllib.request import urlretrieve


#def setup_drake(*, version, build):
#    urlretrieve(
#        f"https://drake-packages.csail.mit.edu/drake/{build}/drake-{version}/#setup_drake_colab.py",
#        "setup_drake_colab.py")
#    import setup_drake_colab
#    setup_drake_colab.setup_drake(version=version, build=build)

import importlib
import json
import os
import shutil
import subprocess
import sys
import warnings
from urllib.request import urlretrieve


def setup_drake(*, version, build='nightly'):
    """Installs drake on Google's Colaboratory and (if necessary) adds the
    installation location to `sys.path`.  This will take approximately two
    minutes, mostly to provision the machine with drake's prerequisites, but
    the server should remain provisioned for 12 hours. Colab may ask you to
    "Reset all runtimes"; say no to save yourself the reinstall.

    Args:
        version: A string to identify which revision of drake to install.
        build: An optional string to specify the hosted directory on
            https://drake-packages.csail.mit.edu/drake/ of the build
            identified by version.  Current options are 'nightly',
            'continuous', or 'experimental'.  Default is 'nightly', which is
            recommended.

    Note: Possible version names vary depending on the build.
        - Nightly builds are versioned by date, e.g., '20200725', and the date
          represents the *morning* (not the prior evening) of the build.  You
          can also use 'latest'.
        - Continuous builds are only available with the version 'latest'.
        - (Advanced) Experimental builds use the version name
          '<timestamp>-<commit>'. See
          https://drake.mit.edu/jenkins#building-binary-packages-on-demand for
          information on obtaining a binary from an experimental branch.

    See https://drake.mit.edu/from_binary.html for more information.

    Note: If you already have pydrake installed to the target location, this
        will confirm that the build/version are the same as the installed
        version, otherwise it will overwrite the previous installation.  If you
        have pydrake available on your ``sys.path`` in a location that is
        different than the target installation, this script will throw an
        Exception to avoid possible confusion.  If you had already imported
        pydrake, this script will throw an assertion to avoid promising that we
        can successfully reload the module.
    """

    assert 'google.colab' in sys.modules, (
        "This script is intended for use on Google Colab only.")
    assert 'pydrake' not in sys.modules, (
        "You have already imported a version of pydrake.  Please choose "
        "'Restart runtime' from the menu to restart with a clean environment.")

    # Check for conflicting pydrake installations.
    v = sys.version_info
    path = f"/opt/drake/lib/python{v.major}.{v.minor}/site-packages"
    spec = importlib.util.find_spec('pydrake')
    if spec is not None and path not in spec.origin:
        raise Exception("Found a conflicting version of pydrake on your "
                        f"sys.path at {spec.origin}.  Please remove it from "
                        "the path to avoid ambiguity.")

    # Check to see if this build/version is already installed.
    setup_version_info = {"version": version, "build": build}
    setup_version_file = "/opt/drake/.setup_drake_colab_token.json"
    already_installed = False
    if os.path.isfile(setup_version_file):
        with open(setup_version_file, "r") as file:
            if json.load(file) == setup_version_info:
                already_installed = True

    # Download the binaries and install.
    if not already_installed:
        if os.path.isdir('/opt/drake'):
            shutil.rmtree('/opt/drake')

        #base_url = 'https://drake-packages.csail.mit.edu/drake/'
        #urlretrieve(f"{base_url}{build}/drake-{version}-bionic.tar.gz",
        #            'drake.tar.gz')

        # THESE ARE A WORKAROUND FOR COLAB WITH PYTHON3.7
        urlretrieve("https://drake-packages.csail.mit.edu/tmp/drake-20210409-pip-snopt-bionic.tar.gz", 'drake.tar.gz')
        subprocess.run(["pip3", "install", "meshcat"])
        # END PYTHON3.7 WORKAROUND

        subprocess.run(['tar', '-xzf', 'drake.tar.gz', '-C', '/opt'],
                       check=True)
        subprocess.run(['apt-get', 'update', '-o',
                        'APT::Acquire::Retries=4', '-qq'], check=True)
        with open("/opt/drake/share/drake/setup/packages-bionic.txt",
                  "r") as f:
            packages = f.read().splitlines()
        subprocess.run(['apt-get', 'install', '-o',
                        'APT::Acquire::Retries=4', '-o', 'Dpkg::Use-Pty=0',
                        '-qy', '--no-install-recommends'] + packages,
                       check=True)

        # Write setup information to disk (so that we can avoid re-running it
        # if the machine is already provisioned).
        with open(setup_version_file, "w") as file:
            json.dump(setup_version_info, file)

    # Check if our new installation is already in the path.
    spec = importlib.util.find_spec('pydrake')
    if spec is None:
        sys.path.append(path)
        spec = importlib.util.find_spec('pydrake')

    # Confirm that we now have pydrake on the path.
    assert spec is not None, (
        "Installation failed.  find_spec('pydrake') returned None.")
    assert path in spec.origin, (
        "Installation failed.  find_spec is locating pydrake, but not in the "
        "expected path.")

    



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
    # run([f"{path}/scripts/setup/ubuntu/18.04/install_prereqs.sh"])

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
