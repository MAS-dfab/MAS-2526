# stick.py
# r: compas >= 2.14.1

import math
from compas.geometry import Point, Vector, Frame, Line, Box, Transformation


def stable_perp(xaxis):
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    def __init__(self, axis: Line, length=None, width=None, depth=None):
        self.axis = axis
        self.length = length or self.DEFAULT_LEN
        self.width = width or self.DEFAULT_SIZE
        self.depth = depth or self.DEFAULT_SIZE
        self.frame = self.compute_frame()

    def compute_frame(self):
        x = self.axis.direction.unitized()
        y = stable_perp(x)
        z = x.cross(y).unitized()
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box

    # ------------------------------
    # AABB COLLISION CHECK
    # ------------------------------
    def compute_aabb(self):
        b = self.geometry
        pts = [Point(x, y, z) for x, y, z in b.vertices]
        xs = [p.x for p in pts]
        ys = [p.y for p in pts]
        zs = [p.z for p in pts]
        return min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)

    def intersects(self, other, clearance=0.0):
        ax0, ax1, ay0, ay1, az0, az1 = self.compute_aabb()
        bx0, bx1, by0, by1, bz0, bz1 = other.compute_aabb()

        ax0 -= clearance; ay0 -= clearance; az0 -= clearance
        ax1 += clearance; ay1 += clearance; az1 += clearance

        return not (
            ax1 < bx0 or bx1 < ax0 or
            ay1 < by0 or by1 < ay0 or
            az1 < bz0 or bz1 < az0
        )
