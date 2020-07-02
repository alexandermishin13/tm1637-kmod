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
* `raw_mode` 0 or 1. 0 is a string mode, 1 is a bytes of segments to light mode.
* `on` on ot off the display (brightness level keeps its level).

```
sysctl dev.tm1637.0.brightness=7
sysctl dev.tm1637.0.on=0
sysctl dev.tm1637.0.raw_mode=1
```

As said above You can switch the driver to one of two modes:
* A `string mode` in which You write to the device up to five chars of digits,
minus and hash signs, spaces and colon followed by "\n". The device ignore
a file position pointer - You do not need to use a `seek()`, but You need to
write a whole string in one write operation.
* A `bytes of segments mode` in which you write up to four bytes of digit
segments to a device. You should also always use `seek()` for the starting
position from which you want to start to write.
No matter what device mode you select, the driver will not re-send unchenged
leading and trailing bytes to the display.

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

## ioctl() functions

For programming languages which have an ability to use of `ioctl()` calls.
`ioctl()` functions defined as:
```
struct tm1637_clock_t {
    int tm_min;
    int tm_hour;
    bool tm_colon;
};

#define TM1637IOC_SET_CLEAR		_IO('T', 1)
#define TM1637IOC_SET_OFF		_IO('T', 2)
#define TM1637IOC_SET_ON		_IO('T', 3)
#define TM1637IOC_SET_BRIGHTNESS	_IOW('T', 11, uint8_t)
#define TM1637IOC_SET_CLOCKPOINT	_IOW('T', 12, uint8_t)
#define TM1637IOC_SET_RAWMODE		_IOW('T', 13, uint8_t)
#define TM1637IOC_SET_SET_CLOCK		_IOW('T', 14 struct tm1637_clock_t)
#define TM1637IOC_GET_RAWMODE		_IOR('T', 23, uint8_t)
```
You could `clear`, `off` and `on` the display just simple as that.
```
dev_fd = open("/dev/tm1637", O_WRONLY);
ioctl(dev_fd, TM1637IOC_DISPLAY_CLEAR);
ioctl(dev_fd, TM1637IOC_DISPLAY_ON);
....
ioctl(dev_fd, TM1637IOC_DISPLAY_OFF);
close(dev_fd);
```

Especially for a time displaing You can use a call of TM1637IOC_SET_CLOCK
ioctl just by coping `tm_hour` and `tm_min` fields from a `struct tm` variable
to a `struct tm1637_clock_t` one.
```
struct tm1637_clock_t cl;
struct tm tm;
time_t rawtime;

time (&rawtime);
localtime_r(&rawtime, &tm);

cl.tm_min = tm.tm_min;
cl.tm_hour = tm.tm_hour;
cl.tm_colon = true;

ioctl(dev_fd, TM1637IOC_SET_CLOCK, &cl);
```
Or use a call of TM1637IOC_SET_CLOCKPOINT ioctl for a clockpoint change only:
```
bool clockpoint;

clockpoint = true;
ioctl(dev_fd, TM1637IOC_SET_CLOCKPOINT, &clockpoint);
```

## Examples

There are examples for Perl and Python in "examples/".

## Bugs

Do not know yet.

## To do

* Check if I could make a `\n` at the end in string mode optional.
* fdt_pinctrl_configure does not work. For now set pins flag forced
befor every start of data transfer..

## Status

Current status of the driver is "alpha".
The driver work tested on Orange Pi PC and Raspberry Pi 2.
