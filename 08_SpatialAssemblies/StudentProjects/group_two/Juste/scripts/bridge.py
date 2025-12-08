# bridge.py
# Cross-family bridging module.
#
# Builds bridges ONLY between Y-family and Z-family sticks
# that are close enough in 3D space and within generation limits.

from compas.geometry import Line
from stick_fixed import Stick


class BridgingModule:

    def __init__(self, stick_list, stick_length, width, depth,
                 bridge_threshold=0.4, max_generations=3):

        self.sticks = stick_list
        self.stick_length = stick_length
        self.width = width
        self.depth = depth

        self.threshold = bridge_threshold
        self.max_gen = max_generations

    # ----------------------------------------------------------
    def _are_cross_family(self, A, B):
        """Bridges only between Y-family and Z-family sticks."""
        return hasattr(A, "family") and hasattr(B, "family") and A.family != B.family

    # ----------------------------------------------------------
    def _gen_level(self, stick):
        """Returns how deep a stick is in the branching tree."""
        parent = getattr(stick, "parent", None)
        level = 0
        while parent is not None:
            level += 1
            parent = getattr(parent, "parent", None)
        return level

    # ----------------------------------------------------------
    def build(self):
        bridges = []

        n = len(self.sticks)

        for i in range(n):
            A = self.sticks[i]
            if not hasattr(A, "family"):
                continue

            for j in range(i + 1, n):
                B = self.sticks[j]
                if not hasattr(B, "family"):
                    continue

                # must be cross-family
                if not self._are_cross_family(A, B):
                    continue

                # generation limits
                if self._gen_level(A) > self.max_gen:
                    continue
                if self._gen_level(B) > self.max_gen:
                    continue

                # spatial threshold
                dist = A.axis.distance_to_line(B.axis)
                if dist > self.threshold:
                    continue

                # build bridging axis between frame origins
                start = A.frame.point
                end = B.frame.point
                axis = Line(start, end)

                bridge = Stick(
                    axis,
                    length=self.stick_length,
                    width=self.width,
                    depth=self.depth,
                    parent_frame=A.frame,   # inherit orientation from A
                )
                bridge.family = "BRIDGE"
                bridge.is_bridge = True
                bridge.parent = A

                bridges.append(bridge)

        return bridges
