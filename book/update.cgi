#!/usr/bin/perl

use strict;
use warnings;

use CGI;
my $r = new CGI;

print $r->header();
print "<p>pulling repo...<br/>";
system 'git fetch origin && git reset --hard origin/master';
system 'git submodule update --init --recursive';
print "<br/>done.</p>";

print "<p>building documentation...<br/>";
chdir "..";

my $status = system('/bin/bash', '-c', '
    source venv/bin/activate &&
    poetry install --only docs &&
    sphinx-build -M html underactuated /tmp/underactuated_doc &&
    rm -rf book/python &&
    cp -r /tmp/underactuated_doc/html book/python
');

if ($status == 0) {
    print "<br/>done.</p>";
} else {
    print "<br/>Error occurred: $status</p>";
}

print $r->end_html;
