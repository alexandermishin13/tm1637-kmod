#!/usr/local/bin/python3.7

import sys, time

colon = True;

if __name__ == "__main__":
    with open('/dev/tm1637', 'w') as clock:

        """ A nice looking clock starting screen for a second
        """
        print("--:--", file=clock, flush=True)
        time.sleep(1)

        while True:
            try:
                """ Write the current time line once per second
                    w/ and w/out a colon alternately
                """
                if colon:
                    print(time.strftime("%H:%M"), file=clock, flush=True)
                else:
                    print(time.strftime("%H %M"), file=clock, flush=True)
                colon = not colon
                time.sleep(1)

            except KeyboardInterrupt:
                """ Blank the display
                """
                print("", file=clock, flush=True)
                sys.exit()
