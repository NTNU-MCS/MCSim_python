#!/usr/bin/perl -w
use strict;
use IO::Socket::IP;

my $service = shift or die "Usage $0 host:port\n";

my $sock=IO::Socket::IP->new(PeerHost =>$service,Type=>SOCK_STREAM); 

$|=1;
while (<$sock>) { print $_; }