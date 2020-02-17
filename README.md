# tm1637 kernel module for FreeBSD

## Installation

You need a kernel source for compile the kld-module.

```
make depend
make
make install
```

## Usage

You can open a character device `/dev/tm1637` and write to it a string about a five characters (format dd:dd).

```
echo "12:30" > /dev/tm1637; sleep 1; echo "12 30" > /dev/tm1637
```

You can change brightness by write a 0...7 digit to kernel variable. 0 is a darkest one.
:
```
sysctl dev.tm1637.0.brightness=7
```


## Bugs

* A clockpoint has just been broken (or not yet implemented). I'm looking for a good idea about an input format;
* You need to close the device before sending a new string (I think so. I didn't check it out)

## Status

Current status of the driver is "unfinished".

