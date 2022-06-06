# point.py

import sys

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = sys.maxsize