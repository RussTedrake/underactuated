"""
Provides setup scripts for provisioning Drake and Underactuated, especially for
use on Google Collaboratory.

For security, this script should never be downloaded and executed without
performing a simple hashsum check, and should always use a pinned Git SHA1.

To compute the hashsum for this script at a given commit::

    git rev-parse --short HEAD  # Ensure your changes are committed!
    sha256sum jupyter_setup.py  # Copy the checksum

This output should then be included in the check of the hashsum when this
script is later downloaded, using ``sha256sum -c``.
"""

import sys
import platform
from IPython import get_ipython

# TODO(eric): Either provide optional checksum of archive here, or figure out
# other provisioning mechanism.

def setup_drake(date):
    """
    Install drake (if necessary) and set up the path. For more information on
    dates, see https://drake.mit.edu/from_binary.html#nightly-releases

    Note:
        At present, this will not check if the existing Drake install is from
        the given date.

    On Google's Colab:
        This will take a minute, but should only need to reinstall once
        every 12 hours. Colab will ask you to "Reset all runtimes", say no to
        save yourself the reinstall.
    """
    try:
        import pydrake
    except ImportError:
        if platform.system() is 'Darwin':
            get_ipython().system(u'if [ ! -d "/opt/drake" ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-{date}-mac.tar.gz && tar -xzf drake.tar.gz -C /opt && /opt/drake/share/drake/setup/install_prereqs; fi'.format(date=date))
        else:
            get_ipython().system(u'if [ ! -d "/opt/drake" ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-{date}-bionic.tar.gz && tar -xzf drake.tar.gz -C /opt && /opt/drake/share/drake/setup/install_prereqs; fi'.format(date=date))
        sys.path.append("/opt/drake/lib/python2.7/site-packages/")

def setup_underactuated(commit, drake_date):
    """
    Install underactuated (if necessary) and set up the path.

    Note:
        At present, this will not check if the existing version is at the given
        commit.

    On Google's Colab:
        This will take a minute, but should only need to reinstall once
        every 12 hours. Colab will ask you to "Reset all runtimes", say no to
        save yourself the reinstall.
    """
    setup_drake(date=drake_date)
    try:
        import underactuated
    except ImportError:
        if platform.system() is 'Darwin':
            get_ipython().system(u'if [ ! -d "/opt/underactuated" ]; then git clone https://github.com/RussTedrake/underactuated.git /opt/underactuated && git checkout {commit} && /opt/underactuated/scripts/setup/mac/install_prereqs; fi'.format(commit=commit))
        else:
            get_ipython().system(u'if [ ! -d "/opt/underactuated" ]; then git clone https://github.com/RussTedrake/underactuated.git /opt/underactuated && git checkout {commit} && /opt/underactuated/scripts/setup/ubuntu/16.04/install_prereqs; fi'.format(commit=commit))
        sys.path.append("/opt/underactuated/src")
