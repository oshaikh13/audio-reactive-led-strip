from __future__ import print_function
from __future__ import division

import platform
import numpy as np
import config
import time

from pySerialTransfer import pySerialTransfer as txfer
link = txfer.SerialTransfer('COM3')
link.open()
time.sleep(2)

_gamma = np.load(config.GAMMA_TABLE_PATH)
"""Gamma lookup table used for nonlinear brightness correction"""

_prev_pixels = np.tile(253, (3, config.N_PIXELS))
"""Pixel values that were most recently displayed on the LED strip"""

pixels = np.tile(1, (3, config.N_PIXELS))
"""Pixel values for the LED strip"""


def _update_esp8266():
    """Sends UDP packets to ESP8266 to update LED strip values

    The ESP8266 will receive and decode the packets to determine what values
    to display on the LED strip. The communication protocol supports LED strips
    with a maximum of 256 LEDs.

    The packet encoding scheme is:
        |i|r|g|b|
    where
        i (0 to 255): Index of LED to change (zero-based)
        r (0 to 255): Red value of LED
        g (0 to 255): Green value of LED
        b (0 to 255): Blue value of LED
    """
    global pixels, _prev_pixels
    # Truncate values and cast to integer

    # print(pixels.shape)
    pixels = np.clip(pixels, 0, 255).astype(int)
    # Optionally apply gamma correc tio
    p = _gamma[pixels] if config.SOFTWARE_GAMMA_CORRECTION else np.copy(pixels)

    m = [0]
    
    for i in range(p.shape[1]):
        
        m.append(p[0][i])  # Pixel red value
        m.append(p[1][i])  # Pixel green value
        m.append(p[2][i])  # Pixel blue value
        if i % 30 == 0:
            # print(m)
            # time.sleep(0.01)
            link.txBuff = m.copy()
            link.send(len(link.txBuff))
            m = [i // 30]
    
    _prev_pixels = np.copy(p)


def update():
    """Updates the LED strip values"""
    _update_esp8266()


# Execute this file to run a LED strand test
# If everything is working, you should see a red, green, and blue pixel scroll
# across the LED strip continously
if __name__ == '__main__':
    import time
    # Turn all pixels off
    pixels *= 0
    pixels[0, 0] = 255  # Set 1st pixel red
    pixels[1, 1] = 255  # Set 2nd pixel green
    pixels[2, 2] = 255  # Set 3rd pixel blue
    print('Starting LED strand test')

    while True:
        pixels = np.roll(pixels, 1, axis=1)
        update()
