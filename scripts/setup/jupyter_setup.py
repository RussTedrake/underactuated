import sys
import platform
from IPython import get_ipython

def setup_drake():
    """ Install drake (if necessary) and set up the path.

        On Google's Colab:
        This will take a minute, but should only need to reinstall once
        every 12 hours. Colab will ask you to "Reset all runtimes", say no to
        save yourself the reinstall.
    """
    try:
        import pydrake
    except ImportError:
        if platform.system() is 'Darwin':
            get_ipython().system(u'if [ ! -d "/opt/drake" ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-mac.tar.gz && tar -xzf drake.tar.gz -C /opt && /opt/drake/share/drake/setup/install_prereqs; fi')
        else:
            get_ipython().system(u'if [ ! -d "/opt/drake" ]; then curl -o drake.tar.gz https://drake-packages.csail.mit.edu/drake/continuous/drake-latest-bionic.tar.gz && tar -xzf drake.tar.gz -C /opt && /opt/drake/share/drake/setup/install_prereqs; fi')
        sys.path.append("/opt/drake/lib/python2.7/site-packages/")

def setup_underactuated():
    """ Install underactuated (if necessary) and set up the path.

        On Google's Colab:
        This will take a minute, but should only need to reinstall once
        every 12 hours. Colab will ask you to "Reset all runtimes", say no to
        save yourself the reinstall.
    """
    setup_drake()
    try:
        import underactuated
    except ImportError:
        if platform.system() is 'Darwin':
            get_ipython().system(u'if [ ! -d "/opt/underactuated" ]; then git clone https://github.com/RussTedrake/underactuated.git /opt/underactuated && underactuated/scripts/setup/mac/install_prereqs; fi')
        else:
            get_ipython().system(u'if [ ! -d "/opt/underactuated" ]; then git clone https://github.com/RussTedrake/underactuated.git /opt/underactuated && underactuated/scripts/setup/ubuntu/16.04/install_prereqs; fi')
        sys.path.append("/opt/underactuated/src")

