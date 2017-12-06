#!/usr/bin/env python

import sys

from PIL import ImageFont

import inkyphat


print("""Inky pHAT: Hello... my name is:

""")

#inkyphat.set_rotation(180)

if len(sys.argv) < 2:
    print("Usage: {} <your name>".format(sys.argv[0]))
    sys.exit(1)

# Show the backdrop image

inkyphat.set_border(inkyphat.RED)
inkyphat.set_image("resources/hello-badge.png")

# Add the text

font = ImageFont.truetype(inkyphat.fonts.AmaticSCBold, 38)

name = sys.argv[1]

w, h = font.getsize(name)

# Center the text and align it with the name strip

x = (inkyphat.WIDTH / 2) - (w / 2)
y = 71 - (h / 2)

inkyphat.text((x, y), name, inkyphat.BLACK, font)

inkyphat.show()
