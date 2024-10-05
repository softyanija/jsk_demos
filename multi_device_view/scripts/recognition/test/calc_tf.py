import argparse
import math

t = 4
h = 100
theta = 45

x = (t + 22.5 * math.sin(math.radians(theta)) + 19 * math.cos(math.radians(theta))) / 1000
y = 9 / 1000
z = (h + 22.5 * math.cos(math.radians(theta)) - 19 * math.sin(math.radians(theta))) / 1000

print("x:{:.6f} y:{:.6f} z:{:.6f}".format(x,y,z))
