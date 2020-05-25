import argparse
import glob
import os

parser = argparse.ArgumentParser(description='Checks my html file links.')
# Add a workspace argument to support non-hermetic testing through `bazel test`.
parser.add_argument('--cwd',
                    default=os.path.dirname(os.path.abspath(__file__)),
                    help='Execute using this current working directory')
args = parser.parse_args()

# Find workspace root by searching parent directories.
os.chdir(args.cwd)
while not os.path.isfile('WORKSPACE.bazel'):
    os.chdir(os.path.dirname(os.getcwd()))


def get_file_as_string(filename):
    f = open(filename, "r")
    s = f.read()
    f.close()
    return s


# Check that all links to code files exist.
for filename in glob.glob("*.html"):
    s = get_file_as_string(filename)

    for tag in ['jupyter', 'pysrcinclude', 'pysrc']:
        index = 0
        while s.find('<' + tag + '>', index) > 0:
            start = s.find('<' + tag + '>', index) + len(tag) + 2
            end = s.find('</' + tag + '>', start)
            index = end + len(tag) + 3
            file = s[start:end]
            if not os.path.exists(file):
                print(filename + " tries to link to the source file " + file +
                      " which doesn't exist")
                exit(-2)
