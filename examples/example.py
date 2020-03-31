#!/usr/local/bin/python3.7

import time

colon = True;

if __name__ == "__main__":
    with open('/dev/tm1637', 'w') as clock:

        """ A nice looking clock starting screen for a second
        """
        clock.seek(0, 0)
        print("--:--", file=clock)
        time.sleep(1)

        while True:
            try:
                """ Write the current time line once per second
                    w/ and w/out a colon alternately
                """
                clock.seek(0, 0)
                if colon:
                    print(time.strftime("%H:%M"), file=clock)
                else:
                    print(time.strftime("%H %M"), file=clock)
                colon = not colon
                time.sleep(1)

            except KeyboardInterrupt:
                """ Blank the display
                """
                clock.seek(0, 0)
                print(" ", file=clock)
                raise
