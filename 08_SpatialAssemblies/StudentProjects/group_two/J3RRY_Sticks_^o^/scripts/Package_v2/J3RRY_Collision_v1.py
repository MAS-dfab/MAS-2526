from compas.geometry import distance_line_line
import math
# from J3RRY_SingleStick_v1 import Stick


class Collision:
    def __init__(self, stick1, stick2):
        """
        Constructor for Collision between two sticks.
        
        Args:
            stick1: type Stick, first stick.
            stick2: type Stick, second stick.
        """
        self.stick1 = stick1
        self.stick2 = stick2

    def aabb_overlap(self):
        """
        Check if AABBs of two sticks overlap.
        
        Returns:
            bool: True if AABBs overlap.
        """
        a_min, a_max = self.stick1.aabb
        b_min, b_max = self.stick2.aabb
        
        return not (
            a_max[0] < b_min[0] or a_min[0] > b_max[0] or
            a_max[1] < b_min[1] or a_min[1] > b_max[1] or
            a_max[2] < b_min[2] or a_min[2] > b_max[2]
        )
    
    @property
    def axis_distance(self):
        """
        Private method to compute the shortest distance between the infinite axes of two sticks.

        Returns:
            float: shortest distance between two infinite axes.
        """
        return distance_line_line(self.stick1.axis, self.stick2.axis, tol=None)
    
    @property
    def segment_distance(self):
        """
        Private method to compute the shortest distance between the line segments of two sticks.

        Returns:
            float: the shortest distance between two line SEGMENTS.
        """
        p1 = self.stick1.axis.start
        q1 = self.stick1.axis.end
        p2 = self.stick2.axis.start
        q2 = self.stick2.axis.end

        def dot(a, b):
            return a.x*b.x + a.y*b.y + a.z*b.z

        def sub(a, b):
            return type(a)(a.x - b.x, a.y - b.y, a.z - b.z)

        def add(a, b):
            return type(a)(a.x + b.x, a.y + b.y, a.z + b.z)

        def mul(a, s):
            return type(a)(a.x * s, a.y * s, a.z * s)

        u = sub(q1, p1)   # segment1 direction
        v = sub(q2, p2)   # segment2 direction
        w = sub(p1, p2)

        a = dot(u, u)     # length^2 of segment1
        b = dot(u, v)
        c = dot(v, v)     # length^2 of segment2
        d = dot(u, w)
        e = dot(v, w)

        D = a * c - b * b
        EPS = 1e-9

        sc, sN, sD = 0.0, 0.0, D
        tc, tN, tD = 0.0, 0.0, D

        # segments almost parallel
        if D < EPS:
            sN = 0.0
            sD = 1.0
            tN = e
            tD = c
        else:
            sN = (b * e - c * d)
            tN = (a * e - b * d)

            if sN < 0.0:
                sN = 0.0
                tN = e
                tD = c
            elif sN > sD:
                sN = sD
                tN = e + b
                tD = c

        if tN < 0.0:
            tN = 0.0
            if -d < 0.0:
                sN = 0.0
            elif -d > a:
                sN = sD
            else:
                sN = -d
                sD = a
        elif tN > tD:
            tN = tD
            if (-d + b) < 0.0:
                sN = 0.0
            elif (-d + b) > a:
                sN = sD
            else:
                sN = (-d + b)
                sD = a

        sc = 0.0 if abs(sN) < EPS else sN / sD
        tc = 0.0 if abs(tN) < EPS else tN / tD

        dP = sub(add(w, mul(u, sc)), mul(v, tc))  # = closest point difference vector
        return math.sqrt(dot(dP, dP))


    def check_collision(self, radius=None):
        """
        Check collision between two sticks using:
        1. AABB --broad-phase
        2. Infinite-line distance --mid-phase
            (temporary, will later replace with segment-segment distance)

        Args:
            radius: collision thickness, default = stick width

        Returns:
            bool: True if collide
        """
        s1 = self.stick1
        s2 = self.stick2
        EPS = 1e-6  # tolerance for floating point comparison

        # 1. AABB
        if not self.aabb_overlap():
            return False

        # 2. Segment distance
        dist = self.segment_distance
        threshold = radius or max(s1.width, s2.width)
        if dist >= threshold - EPS:
            return False
        
        # 3. Parallel sticks check

        return True