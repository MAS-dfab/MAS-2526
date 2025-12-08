# bridge.py
# Bridging module: connects non-coplanar sticks between Y and Z families.

from compas.geometry import Line, Vector, Frame

from stick_fixed import Stick


class BridgingModule(object):
    """
    Build bridging sticks between existing branch sticks.

    Rules:
      - Only between sticks with generation <= max_generation.
      - Only if their axes are non-coplanar:
            abs(dot(x1, x2)) < angle_dot_max
      - Only if minimum distance between axes < distance_threshold.
      - Only if they belong to DIFFERENT families:
            s1.family != s2.family  and each in {'Y', 'Z'}.
    """

    def __init__(
        self,
        stick_list,
        stick_length,
        width,
        depth,
        max_generation=3,
        angle_dot_max=0.75,
        distance_threshold=1000.0,
    ):
        self.stick_list = stick_list
        self.stick_length = float(stick_length)
        self.width = float(width)
        self.depth = float(depth)
        self.max_generation = int(max_generation)
        self.angle_dot_max = float(angle_dot_max)
        self.distance_threshold = float(distance_threshold)

        # optional debug
        self.debug_pairs_accepted = []
        self.debug_pairs_rejected = []

    def build(self):
        bridges = []
        n = len(self.stick_list)

        for i in range(n):
            s1 = self.stick_list[i]
            if s1.generation > self.max_generation:
                continue

            for j in range(i + 1, n):
                s2 = self.stick_list[j]
                if s2.generation > self.max_generation:
                    continue

                # families must exist and be different (Y <-> Z)
                if s1.family not in ("Y", "Z") or s2.family not in ("Y", "Z"):
                    self.debug_pairs_rejected.append((i, j, "no_family"))
                    continue
                if s1.family == s2.family:
                    self.debug_pairs_rejected.append((i, j, "same_family"))
                    continue

                # angle between axes (non-coplanar check)
                dot = abs(s1.frame.xaxis.dot(s2.frame.xaxis))
                if dot >= self.angle_dot_max:
                    self.debug_pairs_rejected.append((i, j, "coplanar"))
                    continue

                # distance between axes
                try:
                    d = s1.axis.distance_to_line(s2.axis)
                except Exception:
                    self.debug_pairs_rejected.append((i, j, "distance_fail"))
                    continue

                if d > self.distance_threshold:
                    self.debug_pairs_rejected.append((i, j, "too_far"))
                    continue

                # build bridge axis between midpoints
                p1 = s1.axis.point_at(0.5)
                p2 = s2.axis.point_at(0.5)
                axis = Line(p1, p2)

                # direction along connection vector
                xdir = Vector.from_start_end(p1, p2)
                if xdir.length < 1e-6:
                    self.debug_pairs_rejected.append((i, j, "degenerate_dir"))
                    continue
                xdir.unitize()

                # frame: xdir, y as stable perp, z = x × y
                up_hint = s1.frame.yaxis
                if abs(up_hint.dot(xdir)) > 0.9:
                    up_hint = s1.frame.zaxis
                zaxis = xdir.cross(up_hint)
                if zaxis.length < 1e-6:
                    zaxis = Vector(0, 0, 1)
                zaxis.unitize()
                yaxis = zaxis.cross(xdir)
                if yaxis.length < 1e-6:
                    yaxis = Vector(0, 1, 0)
                yaxis.unitize()

                mid = axis.point_at(0.5)
                frame = Frame(mid, xdir, yaxis)

                bridge = Stick(
                    axis,
                    length=self.stick_length,
                    width=self.width,
                    depth=self.depth,
                    parent_frame=frame,
                    generation=max(s1.generation, s2.generation) + 1,
                    kind="bridge",
                    family=None,  # bridge is cross-family connector
                )

                bridges.append(bridge)
                self.debug_pairs_accepted.append((i, j))

        return bridges
