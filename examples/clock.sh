#!/bin/sh

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

ctrl_c() {
  # Blank the display and exit
  echo -n "    " > /dev/tm1637/0
  exit
}

# A little show off
echo -n "--:--" > /dev/tm1637/0
sleep 1

# Main loop
while(true); do
  if [ -z ${_COLON} ]; then
    _COLON=':'
  else
    _COLON=' '
  fi
  _TIME=`/bin/date -j "+%H${_COLON}%M"`

  # Send current time string ended up with "\n"  to the display
  echo -n ${_TIME} > /dev/tm1637/0
  sleep 1
done
