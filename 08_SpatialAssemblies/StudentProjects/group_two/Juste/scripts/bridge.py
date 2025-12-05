# bridge.py
# r: compas>=2.14.1

import math
from compas.geometry import Line, Vector

from stick_fixed import Stick


class BridgingModule:
    """
    Bridging engine for RootFrames.

    Takes an existing list of sticks (typically RF.branch_sticks)
    and creates "bridge" sticks between pairs that:

      - are within a maximum distance
      - are not almost parallel (angle > angle_threshold_deg)

    Geometry is entirely COMPAS-based (no Rhino types).
    """

    def __init__(
        self,
        stick_list,
        stick_length=None,
        width=None,
        depth=None,
        max_distance=None,
        angle_threshold_deg=30.0,
    ):
        """
        Parameters
        ----------
        stick_list : list[Stick]
            Input sticks to consider for bridging (usually branch_sticks).
        stick_length : float, optional
            Length of each bridge stick. Defaults to Stick.DEFAULT_LEN
            if not provided.
        width : float, optional
        depth : float, optional
        max_distance : float, optional
            Maximum distance between midpoints of two sticks for them
            to be considered. Defaults to 3 * stick_length.
        angle_threshold_deg : float, optional
            Minimum angle (in degrees) between stick directions for a
            bridge to be created. Pairs that are too parallel are ignored.
        """
        self.sticks = list(stick_list) or []

        self.stick_length = float(stick_length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        self.max_distance = (
            float(max_distance) if max_distance is not None
            else 3.0 * self.stick_length
        )
        self.angle_threshold_deg = float(angle_threshold_deg)

    # ------------------------------------------------------------------
    # internal helpers
    # ------------------------------------------------------------------

    def _midpoint(self, axis):
        return axis.point_at(0.5)

    def _direction(self, stick):
        """Return unit direction vector along the stick (its local xaxis)."""
        d = Vector(
            stick.frame.xaxis.x,
            stick.frame.xaxis.y,
            stick.frame.xaxis.z,
        )
        if d.length < 1e-6:
            # fallback from axis
            d = Vector.from_start_end(stick.axis.start, stick.axis.end)
        if d.length < 1e-6:
            d = Vector(1, 0, 0)
        d.unitize()
        return d

    def _can_bridge(self, sa, sb):
        """Check distance + angular criteria."""
        pa = self._midpoint(sa.axis)
        pb = self._midpoint(sb.axis)

        # distance filter
        dist = pa.distance_to_point(pb)
        if dist > self.max_distance:
            return False

        # angular filter (reject near-parallel)
        da = self._direction(sa)
        db = self._direction(sb)

        dot = max(-1.0, min(1.0, da.dot(db)))
        angle_rad = math.acos(dot)
        angle_deg = math.degrees(angle_rad)

        if angle_deg < self.angle_threshold_deg:
            return False

        return True

    def _build_bridge(self, sa, sb):
        """
        Construct a single bridge stick between two input sticks.

        For now we simply connect the midpoints of the two axes.
        """
        pa = self._midpoint(sa.axis)
        pb = self._midpoint(sb.axis)

        axis = Line(pa, pb)

        # Use axis-based frame (parent_frame=None) so the stick builds
        # a stable orientation from the connection line.
        bridge = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=None,
        )
        return bridge

    # ------------------------------------------------------------------
    # public API
    # ------------------------------------------------------------------

    def build(self):
        """
        Compute all bridges and return a list[Stick].

        This is called from RootFrames.grow_bridging().
        """
        n = len(self.sticks)
        bridges = []

        if n < 2:
            return bridges

        for i in range(n):
            sa = self.sticks[i]
            for j in range(i + 1, n):
                sb = self.sticks[j]

                if not self._can_bridge(sa, sb):
                    continue

                try:
                    bridge = self._build_bridge(sa, sb)
                    bridges.append(bridge)
                except Exception:
                    # keep robust: skip bad pairs silently
                    continue

        return bridges
