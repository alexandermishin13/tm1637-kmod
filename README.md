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
* `--:--` or `-- --` - It is an also possible combination (for a start
decoration).

## Installation

You need a kernel source for compile the kld-module.

Compile and install the kernel driver:
```
make depend
make
sudo make install
```

Compile and install a fdt overlays (edit it for the pins first):
```
cd ./fdt-overlays
make
sudo make install
```

Append an overlay to "/boot/loader.conf":
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

You can open a character device `/dev/tm1637` and write to it a string about
a five characters (format, ##:##).
```
echo "12:30" > /dev/tm1637; sleep 1; echo "## #1" > /dev/tm1637
```
Available symbols are:
* digits [0..9];
* minus sign;
* wildcard '#' for keep this position untouched;
* ':' and ' ' on 3rd position for set or clear clockpoint;
* Any other symbols clears this digit position to blank.

If no ':' nor ' ' on 3rd position the clockpoint keeps untouched.

You can change brightness by write a 0...7 digit to kernel variable. 0 is
a darkest one.
Also You can off or on the display by kernel variable (brightness level keeps
until the display is on again).

```
sysctl dev.tm1637.0.brightness=7
sysctl dev.tm1637.0.on=0
```

## Bugs

* You need to close the device before sending a new string (I think so.
I didn't check it out)

## Status

Current status of the driver is "alpha".
