import curses

# This is an experiment with using the curses module to create a CLI interface.
# https://docs.python.org/2/howto/curses.html

import random
import time

data = ['look', 'at', 'this', 'list']


def main(scr):
    """

    :type scr: curses.WindowObject
    """
    curses.beep()
    scr.nodelay(1)

    titleText = 'Welcome to the OARS Control Panel!'
    scr.addstr(0, (scr.getmaxyx()[1] - len(titleText)) / 2, titleText, curses.A_BOLD)


    for i, v in enumerate(data):
        scr.addstr(i+2, 0, '{}: {}'.format(i, v))
    scr.refresh()

    row = 10

    while True:
        key = scr.getch()
        if key != -1:
            scr.addstr(row, 0, str(key)+'   ')
            row += 1
        else:
            scr.addstr(row, 0, '.'*random.randint(1,3) + '   ')
        time.sleep(.05)

if __name__ == '__main__':
    curses.wrapper(main)