# tm1637 kernel module for FreeBSD

## Installation

You need a kernel source for compile the kld-module.

```
make depend
make
make install
```
Now You can load the module:
```
kldload tm1637
```

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

* You need to close the device before sending a new string (I think so. I didn't check it out)

## Status

Current status of the driver is "early alpha".

