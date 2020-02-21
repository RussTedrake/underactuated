import argparse
import glob
import os

parser = argparse.ArgumentParser(description='Checks my html file links.')
# Add a workspace argument to support non-hermetic testing through `bazel test`.
parser.add_argument('--workspace', help='Path to the WORKSPACE root.')
args = parser.parse_args()


def get_file_as_string(filename):
    f = open(filename, "r")
    s = f.read()
    f.close()
    return s


# Check that all links to code files exist.
for filename in glob.glob("*.html"):
    print(filename)
    s = get_file_as_string(filename)

    for tag in ['jupyter', 'pysrcinclude', 'pysrc']:
        index = 0
        while s.find('<' + tag + '>', index) > 0:
            start = s.find('<' + tag + '>', index) + len(tag) + 2
            end = s.find('</' + tag + '>', start)
            index = end + len(tag) + 3
            file = s[start:end]
            if tag != 'jupyter':
                file = os.path.join('underactuated', file)
            if args.workspace:
                file = os.path.join(args.workspace, file)
            if not os.path.exists(file):
                print(os.environ)
                print(filename + " tries to link to the source file " + file +
                      " which doesn't exist")
                exit(-2)
