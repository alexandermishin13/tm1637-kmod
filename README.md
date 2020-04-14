# tm1637 kernel character device for FreeBSD

![TM1637](/tm1637.jpg?raw=true "TM1637 display")

## About

There are not so many drivers for tm1637 on FreeBSD or Linux
(There is such one: https://gitlab.com/alexandermishin13/tm1637-display).
But even if you find one, it will not be easy to use in your project in
languages other than c/c++. It is sad as the tm1637 display is a very
simple to understand device. An idea was to make a kernel character device,
suitable for writing strings to it, which would be displayed by it
immediately. I wished the device was smart enough for minimize a transfer
a characters which is not changed. As example, You can send:

* `12:30` - 4 digits would be transferred by kernel to tm1637
* `12 31` - Only "231" would be tranferred. As colon is a part of second
digit (right now is "2") and colon is changed, then second digit is also in
a changed digits list;
* `12 31` or `1231` - No numbers will be transferred at all, since neither
digits nor colon not changed
* `12:31` - Only second digit (see above) will be transferred now;
* `## ##` - Only second digit again, but even faster than previous one as no
other digits compares would be made at all;
* `  :  ` or ` ` - In 2nd case trailing spaces will be added;
* `--:--` or `-- --` - It is an also possible combination (for a start
decoration).
All that combinations **should be** ended up with '\n'.
`echo` do it for You automatically

## Installation

You need a kernel source for compile the kld-module. I have the source tree
as a nfs mounted filesystem to `/usr/src`. It may be also an usb-flash with
it as the kernel sources are not often needed.

You can uncomment the line with -DDEBUG in the `Makefile` before compilation
to control how the driver processes strings, what it sends to `tm1637`.
Compile and install the kernel driver:
```
make depend
make
sudo make install
```

Compile and install a fdt overlays You need it for declare `tm1637` device.
Edit the overlay source for the a pins definition first:
```
cd ./fdt-overlays
make
sudo make install
```

Append an overlay to "/boot/loader.conf" and reboot:
<pre><code>
fdt_overlays="sun8i-h3-sid,sun8i-h3-ths<b>,sun8i-h3-tm1637-gpio</b>"
</code></pre>

Now You can load the module:
```
kldload tm1637
```

After loading the module a character device "/dev/tm1637" and a sysctl-tree
branch "dev.tm1637.0" will be created. Right now the display is dark and
empty as it contains all blanks w/out a colon.

Now the display can be easy used from PHP or Python or anything alse.

## Usage

You can change by write to a kernel variable:
* `brightness` 0...7 digit, 0 is a darkest one.
* `mode` 0 or 1. 0 is a string mode, 1 is a bytes of segments to light mode.
* `on` on ot off the display (brightness level keeps its level).

```
sysctl dev.tm1637.0.brightness=7
sysctl dev.tm1637.0.on=0
sysctl dev.tm1637.0.mode=1
```

As said above You can switch the driver to one of two modes:
* A `string mode`, when You write to the device up to five chars of digits,
minus and hash signs, spaces and colon followed by "\n". You need to seek
to zero or close and reopen device any time before You can write a new string.
(/bin/echo do it any time).
* A `bytes of segments mode`, when You write to the device up to four bytes of
segments to light and also need to seek any time but to any of four byte
positons.
No matter what device mode you select, the driver will not send leading and
trailing unchanged bytes to the display.

### String mode

You can open a character device `/dev/tm1637` and write to it a string about
a five characters (format, ##:##).
```
echo "12:30" > /dev/tm1637; sleep 1; echo "## #1" > /dev/tm1637
```
Available symbols are:
* digits [0..9];
* minus sign;
* space;
* placeholder '#' for keep this position untouched;
* ':' and ' ' in 3rd position for set or clear clockpoint;
* Any other symbols leads to an error message.

If at the 3rd position there is neither ':' nor ' ' (number format),
then the clockpoint will be turned off.
On the other hand, the clock format is strict and requires 5 characters:
a colon or space in the third position and 4 digits, spaces, minus signs
or placeholders in other ones ('##:##').

### Bytes of segments mode

You can write up to four bytes of seven segments (for example: `0x06` is a "1"
and `0x7f` is a "8") w/o "\n" at the end. For a colon in the middle of display
set an eight bit of second byte and unset it for turn the colon off. You can
write all four byte at once or set a position by seek() and write one or more
bytes.

## Examples

There are examples for Perl and Python in "examples/".

## Bugs

Do not know yet.

## To do

* To add optional default parameters for a brightness and a mode to fdt-overlays.
* To check if I could reset to zero a file pointer position after a string 
successful write (Using a seek() by driver itself).

## Status

Current status of the driver is "alpha".
The driver work tested on Orange Pi PC and Raspberry Pi 2.
