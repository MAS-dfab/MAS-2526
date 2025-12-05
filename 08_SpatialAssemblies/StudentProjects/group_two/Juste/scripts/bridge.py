# bridge.py

from stick_fixed import Stick
from compas.geometry import Line, Vector



def detect_bridging_candidates(sticks, threshold=9999999):
    """Return index pairs (i, j) of sticks that are nearest and non-coplanar."""
    pairs = []
    for i in range(len(sticks)):
        for j in range(i+1, len(sticks)):
            # skip if nearly coplanar by direction-dot
            d = abs(sticks[i].frame.zaxis.dot(sticks[j].frame.zaxis))
            if d > 0.98:
                continue
            pairs.append((i, j))
    return pairs


class BridgingModule:
    def __init__(self, stick_list, stick_length, width, depth):
        self.sticks = stick_list
        self.stick_length = stick_length
        self.width = width
        self.depth = depth
        self.bridges = []

    def build_bridge(self, parent, target):
        c0 = parent.frame.point
        c1 = target.frame.point

        n0 = parent.frame.zaxis.unitized()
        n1 = target.frame.zaxis.unitized()

        mid = Point(
            0.5 * (c0.x + c1.x),
            0.5 * (c0.y + c1.y),
            0.5 * (c0.z + c1.z),
        )

        # child 1
        d0 = (mid - c0).unitized()
        y0 = n0
        x0 = d0
        z0 = x0.cross(y0).unitized()

        start0 = c0
        end0   = c0 + x0 * self.stick_length
        axis0  = Line(start0, end0)
        s0 = Stick(axis0, self.stick_length, self.width, self.depth)

        # child 2
        d1 = (mid - c1).unitized()
        y1 = n1
        x1 = d1
        z1 = x1.cross(y1).unitized()

        start1 = c1
        end1   = c1 + x1 * self.stick_length
        axis1  = Line(start1, end1)
        s1 = Stick(axis1, self.stick_length, self.width, self.depth)

        return [s0, s1]

    def build(self):
        pairs = detect_bridging_candidates(self.sticks)
        for i, j in pairs:
            bpair = self.build_bridge(self.sticks[i], self.sticks[j])
            self.bridges.extend(bpair)
        return self.bridges
