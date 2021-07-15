# tm1637 kernel character device for FreeBSD

![TM1637](/tm1637.jpg?raw=true "TM1637 display")

## About

There are not so many libraries for tm1637 on FreeBSD nor Linux
(There is such one: https://gitlab.com/alexandermishin13/tm1637-display).
But even if you find one, it will not be easy to use in your project in
languages other than c/c++. It is sad as the tm1637 display is a very
simple to understand device. An idea was to make a kernel character device,
suitable for writing strings to it, which would be displayed by it
immediately. 
As example, You can send:

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

Use `echo -n` for ommit a trailing `\n` character.

## Installation

You need a kernel source for compile the kld-module. I have the source tree
as a nfs mounted filesystem to `/usr/src`. It may be also an usb-flash with
it as the kernel sources are not often needed.

You can uncomment the line with -DDEBUG in the `Makefile` before compilation
to control how the driver processes strings, what it sends to `tm1637`.
Compile and install the kernel driver:
```shell
% make depend
% make
% sudo make install
```

Compile and install a fdt overlays You need it for declare `tm1637` device.
Edit the overlay source for the a pins definition first:
```shell
% cd ./fdt-overlays
% make
% sudo make install
```

Append an overlay to "/boot/loader.conf" and reboot:
<pre><code>
fdt_overlays="sun8i-h3-sid,sun8i-h3-ths<b>,sun8i-h3-tm1637-gpio</b>"
</code></pre>

Now You can load the module:
```shell
% kldload tm1637
```

After loading the module a character device "/dev/tm1637" and a sysctl-tree
branch "dev.tm1637.0" will be created. Right now the display is dark and
empty as it contains all blanks w/out a colon.

Now the display can be easy used from PHP or Python or anything alse.

## Usage

You can change by write to a kernel variable:
* `brightness` 0...7 digit, 0 is a darkest one. Can be set by any user;
* `on` on ot off the display (brightness level keeps its value). Can be set by
any user.

```shell
% sysctl dev.tm1637.0.brightness=7
% sysctl dev.tm1637.0.on=0
```

You can send an up to five characters string of digits or an up to six coded
characters string of a raw command to the display chip.

### String of digits

You can open a character device `/dev/tm1637` and write to it a string about
a five characters (format, ##:##).
```shell
% echo -n "12:30" > /dev/tm1637/0; sleep 1; echo -n "12 30" > /dev/tm1637
```
Available symbols are:
* digits [0..9];
* minus sign;
* space;
* placeholder '#' for keep this position untouched;
* ':' and ' ' in 3rd position sets or clears clockpoint;
* Any other symbols leads to an error message.

If at the 3rd position there is neither ':' nor ' ' (number format),
then the clockpoint will be turned off.
On the other hand, the clock format is strict and requires 5 characters:
a colon or space in the third position and 4 digits, spaces, minus signs
or placeholders in other ones ('##:##').

All other symbols (other combinations of display segments) can be drawn by
sending of strings of coded characters formatted as `tm1637` chip commands.

### String of coded characters

You can write up to six coded characters as a command. In this case, the driver
gives you the ability to program the chip directly, using a sequence of control
bytes, taking over the entire task of transferring data bits, including stop
and start bits.

The driver determines this mode by the very first character of the line, if it
is not a digit, but a command, such as 0x40,0x44,0x80,0x88-0x8f. 

By example:
```shell
% printf $‘\x80’ > /dev/tm1637/0
% printf $‘\x8b’ > /dev/tm1637/0
% printf $‘\x44\xc1\x86’ > /dev/tm1637/0
```
* First command turns the display off;
* Second one turns it on and set a brightness level to 3 (0x8b = 0x88 & 0x03);
* Third one lights the second position (0xc1 = 0xc0 & 0x01) segments to display
a digit `1` followed by a colon (0x86 = 0x06 & 0x80).
Bit masks for segments range from `0x00` (all segments are dark) to `0x7f` (all
segments are lit).
For a colon in the middle of the display set an eighth bit of the second data
byte, or clear it for turn the colon off (See example above).
You can also write all four byte at once by a command `0x40` followed by
address `0xc0` and four bytes of bitmasks (See `tm1637` datasheet).

## ioctl() functions

For programming languages which have an ability to use of `ioctl()` calls.
`ioctl()` functions defined in **/usr/include/dev/tm1637/tm1637.h**:
```c
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
#define TM1637IOC_SET_SET_CLOCK		_IOW('T', 14, struct tm1637_clock_t)
```
You could `clear`, `off` and `on` the display just simple as that.
```c
dev_fd = open("/dev/tm1637/0", O_WRONLY);
ioctl(dev_fd, TM1637IOC_DISPLAY_CLEAR);
ioctl(dev_fd, TM1637IOC_DISPLAY_ON);
....
ioctl(dev_fd, TM1637IOC_DISPLAY_OFF);
close(dev_fd);
```

Especially for a time displaing You can use a call of TM1637IOC_SET_CLOCK
ioctl just by coping `tm_hour` and `tm_min` fields from a `struct tm` variable
to a `struct tm1637_clock_t` one.
```c
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
```c
bool clockpoint;

clockpoint = true;
ioctl(dev_fd, TM1637IOC_SET_CLOCKPOINT, &clockpoint);
```

## Examples

There are examples for Perl, Python and shell in "examples/".

## Bugs

Do not know yet.

## Status

The driver is tested on Orange Pi PC and Raspberry Pi 2.

## A couple of words

There is another driver for `tm1637` displays -
[`cuse`-based one](https://gitlab.com/alexandermishin13/tm1637-cuse).
It does not require the kernel source to compile, but requires a loaded
kernel module `cuse.ko` to work.
