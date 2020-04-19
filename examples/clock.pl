#!/usr/local/bin/perl

use strict;

my $dots = 0;

# Off the display and close device
sub signal_handler {
    print C "\n"; # Trailing spaces will be added
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
    if ($dots == 0) {
      printf C "%02d %02d\n", $hour, $min;
    }
    else {
      printf C "%02d:%02d\n", $hour, $min;
    }
    $dots = abs(--$dots);
    sleep(1);
}

# Always close the device before exit.
END {
    close C;
}
