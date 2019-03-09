#!/usr/bin/perl

use strict;
use warnings;

use CGI;
my $r = new CGI;

print $r->header();
print "pulling repo...<br/>";
system 'git fetch origin && git reset --hard origin/master';
system 'git submodule update --init --recursive';
print "<br/>done.";
print $r->end_html;
