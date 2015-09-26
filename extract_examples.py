from bs4 import BeautifulSoup
from collections import defaultdict
import re
import shutil
import os


"""
extract_examples: pull all the Matlab code examples out of a textbook html
file, and create matching unit tests.

This script will attempt to extract every <code> element which has a
"testfile=" attribute. For each matching element, it will create a
corresponding file using the "testfile" as the filename. Matlab scripts will
be wrapped as functions, while class definitions will be preserved exactly.
The result should be a set of classes and functions suitable for testing using
the Drake unitTest system.

Usage:
    python extract_examples.py --help

Requirements:
    python (2.7)
    Beautiful Soup 4:
        sudo apt-get install python-bs4
        OR
        pip install beautifulsoup4
    html5lib:
        sudo apt-get install python-html5lib
        OR
        pip install html5lib
"""

class CodeExample(object):
    extension_map = {"matlab": ".m"}
    def __init__(self, name, code, chapter_title, language="matlab"):
        self.name = name
        self.code = code
        self.chapter_title = chapter_title
        self.language = language

    def folder(self):
        return self.chapter_title

    def filename(self):
        return self.name + self.extension_map[self.language]

    def path(self):
        return os.path.join(self.folder(), self.filename())

    def writeToFile(self, dir=os.path.abspath(os.path.curdir)):
        if not os.path.exists(os.path.join(dir, self.folder())):
            os.mkdir(os.path.join(dir, self.folder()))
        with open(os.path.join(dir, self.path()), 'w') as f:
            if not self.code.lstrip().startswith("classdef"):
                f.write("function {:s}\n".format(self.name))
            f.write(self.code)
            f.write("\n")

def chapter_name(chapter):
    """Return a sanitized version of the next h1 tag's contents"""
    h1 = chapter.find("h1")
    if h1 is not None:
        s = unicode(h1.string).lower()
        s = re.sub(r"[^0-9a-z]", "_", s)
        s = re.sub(r"_+", "_", s)
        return s
    else:
        raise ValueError("Cannot find an h1 for this chapter")

def find_chapter_numbers(soup):
    chapter_numbers = {}
    appendix_numbers = {}
    next_chapter = 1
    next_appendix = 1
    for chapter in soup.find_all("section", "chapter"):
        if chapter.find_parents("appendix"):
            appendix_numbers[chapter_name(chapter)] = next_appendix
            next_appendix += 1
        else:
            chapter_numbers[chapter_name(chapter)] = next_chapter
            next_chapter += 1
    return chapter_numbers, appendix_numbers

def parse_examples(code_elements, chapter_numbers, appendix_numbers):
    folder_counts = defaultdict(lambda: 0)
    for el in code_elements:
        if "testfile" not in el.attrs:
            continue
        ch_name = chapter_name(el.find_parent("section", "chapter"))
        if ch_name in chapter_numbers:
            chapter_title = "Chapter_{:02d}_{:s}".format(chapter_numbers[ch_name], ch_name)
        elif ch_name in appendix_numbers:
            chapter_title = "Appendix_{:s}_{:s}".format(chr(appendix_numbers[ch_name] + 64), ch_name)
        code = unicode(el.string)
        folder_counts[ch_name] += 1
        count = folder_counts[ch_name]
        name = el["testfile"]
        example = CodeExample(name, code, chapter_title)
        yield example

def collect_examples(soup):
    chapter_numbers, appendix_numbers = find_chapter_numbers(soup)
    examples = parse_examples(soup.find_all("code"), chapter_numbers, appendix_numbers)
    return examples

def extract_and_write_examples(textbook_html_file, destination_dir):
    """
    Extract all the matlab code snippets which have a 'testfile' attribute
    and write them to the appropriate files within destination_dir.
    """
    build_path = os.path.abspath(destination_dir)
    if os.path.exists(build_path):
        shutil.rmtree(build_path)
    os.makedirs(build_path)

    soup = BeautifulSoup(open(textbook_html_file), "html5lib")
    for example in collect_examples(soup):
        example.writeToFile(build_path)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Extract all the examples labeled with a 'testfile' attribute, and use them to create unit tests.")
    parser.add_argument("textbook_html_file", type=str, nargs=1)
    parser.add_argument("destination_dir", type=str, nargs="?", default=os.path.join(os.path.curdir, 'examples'), help="where to put the resulting unit tests (default: ./examples)")
    args = parser.parse_args()
    extract_and_write_examples(args.textbook_html_file[0], args.destination_dir)
