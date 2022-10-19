#!/usr/bin/perl -Tw
use strict;
use bytes;

$|=1;

use Crypt::OpenSSL::RSA;
use MIME::Base64;

my $TALKER = 'U9SEC';

my %key = ();
my %assembly = (); 

while (my $message = <STDIN>) {

    # Parse the message
    #
    next unless $message =~ /^\!$TALKER,(\d),(\d),(\d),(.*)\r\n$/; # Max 4096 bit key length 
    my ($size, $index, $parcel, $chunk) = ($1, $2, $3, $4);

    # Assemble parcels
    #
    # ##? Add some warnings/checks about duplicates, index > size etc
    #
    $assembly{$parcel}{$index - 1} = $chunk;  ##? JAM: Could be a list of all attempts 
    next if int(keys %{$assembly{$parcel}}) < $size;
    #
    # Assemble a letter from a completed parcel
    my $letter = "";
    for (my $part = 0; $part < $size; $part++) {
	$letter .= $assembly{$parcel}{$part};
	##? JAM: Try all combinations, next on no valid (require decoding inside the loop)
    }

    # Decode the parcel
    #
    # Get the signers key
    next unless $letter =~ /^([a-zA-z0-9\/_ .-]+);(.*)$/;
    my($signer, $content) = ($1, $2);
    unless (exists $key{$signer}) {
	#
	# Where we expect to find the key
	my $keyfile = $signer;
	$keyfile =~s/[^a-zA-Z0-9]/_/g;
	$keyfile = "$keyfile.key.pub";
	#
	if (open KEY, '<', $keyfile) {
	    $key{$signer} = join('', <KEY>);
	    close(KEY);
	}
    }
    next unless $key{$signer};
    #
    # Decode encrypted content
    my $rsa = Crypt::OpenSSL::RSA->new_public_key($key{$signer});
    $rsa->use_pkcs1_padding;
    $content = $rsa->public_decrypt(decode_base64($content));
    #
    # Validate the content 
    next unless $content =~ /;[!\$].*\r\n/s;

    # Clear the parcel slot and print
    #
    delete($assembly{$parcel});
    print "$signer;$content";
}