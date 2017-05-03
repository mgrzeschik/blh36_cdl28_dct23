# mouse_positioning.py
# Chris Leary -- 04/29/07
# A demonstration of how to position a mouse at a particular location on the screen, either 
# absolutely or relatively, on *NIX in Python, using Xlib (via the python-xlib package)
# The use of teh_display as a variable is to differentiate it from the display module.


import os
from Xlib import display
import serial


def c2(num):
    if num & 0x80: return -((~num & 0xff) + 1)
    return num


def absolutely_position_pointer(teh_display, abs_x, abs_y):
    # Get the screen that the display is currently using.
    current_screen = teh_display.screen()
    # Warp the pointer to the absolute value provided -- no questions asked!
    current_screen.root.warp_pointer(abs_x, abs_y)
    teh_display.sync()


def relatively_position_pointer(teh_display, delta_x, delta_y):
    # Get the screen that the display is currently using.
    current_screen = teh_display.screen()

    # Query the current (absolute) x and y coordinates within the screen.
    pointer_query_result = current_screen.root.query_pointer()
    current_x = pointer_query_result._data['root_x']
    current_y = pointer_query_result._data['root_y']

    # Position the pointer at its newly calculated location.
    absolutely_position_pointer(teh_display, current_x + delta_x, current_y + delta_y)


if __name__ == '__main__':
    ENV_DISPLAY = os.environ.get("DISPLAY")
    print 'Using display: ', ENV_DISPLAY
    current_display = display.Display(ENV_DISPLAY)
    
    # Set up the serial device.
    sp = serial.Serial('/dev/ttyUSB1')
    print sp
    while True:
        mouse_output = sp.readline().strip()
        mouse_output = mouse_output[12:24]
        if mouse_output == '000000000000': continue

        

        # first seven bits are always one
        # eighth bit always zero
        # last two bits are always one
        # seven bits left:
        #   bit two of third nibble cleared on left click
        #   bit 
        print mouse_output
        try: mouse_output = eval('0x' + mouse_output)
        except: continue
        # 0x2 never set
        # 0x4 never set
        # 0x8 never set
        # 0x10 never set
        # 0x20 never set
        # 0x40 never set
        # 0x80 never set
        # 0x100 movement in both directions
        # 0x200 movement in both directions
        # 0x400 lateral movement
        # 0x800 movement in both directions
        # 0x1000 movement in both directions
        # 0x2000
        # 0x4000

        left_down = mouse_output & 0x400
        right_down = (mouse_output & 0xc000) == 0x4000

        if left_down: print 'Left mouse down.'
        if right_down: print 'Right mouse down.'
        print hex(mouse_output), '\n'

        """if left_down: #button == 'fe27':
            # 1111 1110 0010 0111
            print 'Left button down.'
        elif button == 'feeb':
            # 1111 1110 1110 1011
            print 'Middle button down.'
        elif button == 'fe63':
            # 1111 1110 0110 0011
            print 'Right button down.'
        elif button == 'ffdb':
            # 1111 1110 1101 1011
            print 'Button up.'
        else:
            vertical_msb = mouse_output[6:8]
            vertical_lsb = mouse_output[8:10]
            horizontal_msb = mouse_output[:2]
            #horizontal_lsb = mouse_output[
            print mouse_output[2:4], mouse_output[4:6], mouse_output[8:10], mouse_output[10:12]"""

        # up and down seems to be mouse_output[2:4]
        # left and right seems to be mouse_output[:2]

        # 03f f80 is moving up the tiniest bit, no lateral movement
        # ff8 081 is moving to the left the tiniest bit
        # 010 000 is moving to the right the tiniest bit
        # 030 100 is moving to the bottom the tiniest bit
