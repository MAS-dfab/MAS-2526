# bridge.py
# r: compas>=2.14.1

import math
from compas.geometry import Line, Vector

from stick_fixed import Stick


class BridgingModule:
    """
    Bridging module:

    - Takes an existing list of branch sticks.
    - Looks for pairs that are close enough and not nearly parallel.
    - Builds bridge sticks between their midpoints.

    Density-awareness:
        Because we only bridge when sticks are within `max_distance`,
        bridges naturally appear in denser regions of the field.
    """

    def __init__(
        self,
        stick_list,
        stick_length=None,
        width=None,
        depth=None,
        max_distance=None,
        min_angle_deg=15.0,
    ):
        self.sticks = stick_list
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.width = width or Stick.DEFAULT_SIZE
        self.depth = depth or Stick.DEFAULT_SIZE
        self.max_distance = float(max_distance) if max_distance is not None else None
        self.min_angle_rad = math.radians(float(min_angle_deg))

    # ------------------------------------------------------------------ #
    # internal: tests & construction                                     #
    # ------------------------------------------------------------------ #

    def _can_bridge(self, sa, sb):
        """Test if two sticks are eligible for bridging."""
        # 1) distance between midpoints
        pa = sa.axis.point_at(0.5)
        pb = sb.axis.point_at(0.5)
        d = pa.distance_to_point(pb)

        if self.max_distance is not None and d > self.max_distance:
            return False

        # 2) not already intersecting / too close
        if sa.intersects(sb, clearance=0.0):
            return False

        # 3) ensure they are not nearly parallel
        xa = sa.frame.xaxis.unitized()
        xb = sb.frame.xaxis.unitized()
        angle = xa.angle(xb)
        if angle < self.min_angle_rad:
            return False

        return True

    def _build_bridge(self, sa, sb):
        """Construct a single bridge stick between midpoints of sa, sb."""
        pa = sa.axis.point_at(0.5)
        pb = sb.axis.point_at(0.5)

        axis = Line(pa, pb)

        # orient bridge using first parent's frame as base
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=sa.frame,
        )
        return child

    # ------------------------------------------------------------------ #
    # public: build all bridges                                          #
    # ------------------------------------------------------------------ #

    def build(self):
        bridges = []
        n = len(self.sticks)

        for i in range(n):
            sa = self.sticks[i]
            for j in range(i + 1, n):
                sb = self.sticks[j]

                if not self._can_bridge(sa, sb):
                    continue

                bridges.append(self._build_bridge(sa, sb))

        return bridges
