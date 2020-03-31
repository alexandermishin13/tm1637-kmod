#!/usr/local/bin/perl

use strict;

my $dots = 0;

# Off the display and close device
sub signal_handler {
    seek C, 0, 0;
    print C " \n"; # Will be trailed by spaces
    close C;
    exit;
};

open C, ">/dev/tm1637" or die "Reason: $!";

# Control+C handler (see above)
$SIG{INT} = \&signal_handler;

# Unbuffered output
select(C);
$|=1;

# A little show off
print C "--:--\n";
sleep(1);

# Main loop
while(1) {
    my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) = localtime();
    seek C, 0, 0;
    if ($dots == 0) {
      printf C "%02s %02s\n", $hour, $min;
    }
    else {
      printf C "%02s:%02s\n", $hour, $min;
    }
    $dots = abs(--$dots);
    sleep(1);
}
close C;
