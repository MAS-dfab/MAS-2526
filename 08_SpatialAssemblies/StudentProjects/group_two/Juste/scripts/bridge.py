# bridge.py
# BridgingModule for COMPAS-based spatial L-system
# Implements:
#   - Cross-family bridging (Y-family ↔ Z-family)
#   - Non-coplanar check
#   - Distance thresholds
#   - Max bridging depth = 3 generations
#   - Same offset and tangent/normal blending rules as branching

import math
from compas.geometry import Line, Vector, distance_point_point

from stick_fixed import Stick


# Family definitions (same as branch.py)
Y_FACES = [0, 2]
Z_FACES = [1, 3]


def classify_family(stick):
    """
    Determine if stick belongs to Y-family or Z-family based on its frame.

    Rule:
        If |dot(tangent, world Z)| > |dot(tangent, world Y)| → call it Z-family
        Otherwise Y-family

    This works because:
        - Y-family sticks tend to grow in ±Y directions
        - Z-family sticks tend to grow in ±Z directions
    """
    t = stick.frame.xaxis.unitized()

    world_y = Vector(0, 1, 0)
    world_z = Vector(0, 0, 1)

    dy = abs(t.dot(world_y))
    dz = abs(t.dot(world_z))

    return "Z" if dz > dy else "Y"


def are_non_coplanar(s1, s2, threshold=0.95):
    """
    Non-coplanar check:

        dot(tangents) < threshold

    threshold = 0.95 → angle > ~18° required
    """
    t1 = s1.frame.xaxis.unitized()
    t2 = s2.frame.xaxis.unitized()
    return abs(t1.dot(t2)) < threshold


class BridgingModule:
    """
    Given list of branch sticks, generate bridge sticks:

      • Only between Y-family and Z-family
      • Only if non-coplanar
      • Only if distance < bridge_threshold
      • Max depth = 3 generations

    Bridges are created as COMPAS Box-based Stick objects.
    """

    def __init__(
        self,
        stick_list,
        stick_length,
        width,
        depth,
        bridge_threshold=200.0,
        coplanar_threshold=0.95,
        max_generations=3,
    ):
        self.sticks = stick_list

        self.stick_length = stick_length
        self.width = width
        self.depth = depth

        self.bridge_threshold = float(bridge_threshold)
        self.coplanar_threshold = float(coplanar_threshold)
        self.max_generations = int(max_generations)

    # ------------------------------------------------------------------
    # INTERNAL CHILD BUILDER
    # ------------------------------------------------------------------

    def _build_bridge_stick(self, s1, s2):
        """
        Build a bridge stick between two parent sticks.

        The bridge axis is the segment connecting their closest points.
        Orientation = inherits frame from s1 (arbitrary but stable).
        """
        # Compute endpoints
        p1 = s1.axis.point_at(0.5)
        p2 = s2.axis.point_at(0.5)

        axis = Line(p1, p2)

        # Parent family determines which dimension offsets
        parent_frame = s1.frame

        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            parent_frame=parent_frame,
        )
        return child

    # ------------------------------------------------------------------
    # MAIN BRIDGING STEP
    # ------------------------------------------------------------------

    def build(self):
        """
        Build all bridges up to max_generations.
        """

        # Classify sticks into families
        families = {s: classify_family(s) for s in self.sticks}

        bridges = []
        generation = 0

        while generation < self.max_generations:
            new_bridges = []

            for i, s1 in enumerate(self.sticks):
                f1 = families[s1]

                # target family
                target = "Z" if f1 == "Y" else "Y"

                for j, s2 in enumerate(self.sticks):
                    if i == j:
                        continue

                    if families[s2] != target:
                        continue

                    # Distance test
                    p1 = s1.axis.point_at(0.5)
                    p2 = s2.axis.point_at(0.5)
                    d = distance_point_point(p1, p2)

                    if d > self.bridge_threshold:
                        continue

                    # Non-coplanar test
                    if not are_non_coplanar(s1, s2, threshold=self.coplanar_threshold):
                        continue

                    # Build bridge
                    b = self._build_bridge_stick(s1, s2)
                    new_bridges.append(b)

            if not new_bridges:
                break

            bridges.extend(new_bridges)
            self.sticks.extend(new_bridges)

            generation += 1

        return bridges
