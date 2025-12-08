# stick_fixed.py
# Lightweight Stick class using COMPAS geometry only.
# Geometry (Rhino breps) is built later in the GH script.

from compas.geometry import Point, Vector, Line, Frame


def build_frame_from_axis(axis, up_hint=None):
    """
    Build a stable orthonormal frame from a Line axis.

    axis    : compas.geometry.Line
    up_hint : optional Vector used to resolve perpendicular direction
    """
    start = axis.start
    end = axis.end

    # X-axis along the stick
    xaxis = Vector.from_start_end(start, end)
    if xaxis.length < 1e-6:
        xaxis = Vector(1, 0, 0)
    xaxis.unitize()

    # Perpendicular Y-axis
    if up_hint is None:
        up_hint = Vector(0, 0, 1)

    yaxis = up_hint.cross(xaxis)
    if yaxis.length < 1e-6:
        # if parallel to up_hint, pick another vector
        yaxis = Vector(0, 1, 0).cross(xaxis)

    if yaxis.length < 1e-6:
        yaxis = Vector(0, 1, 0)

    yaxis.unitize()

    origin = axis.point_at(0.5)
    return Frame(origin, xaxis, yaxis)


class Stick(object):
    """
    Minimal stick representation:

      - axis   : compas Line (centerline)
      - frame  : compas Frame (orientation)
      - length : float (along frame.xaxis)
      - width  : float (along frame.yaxis)
      - depth  : float (along frame.zaxis)

      - generation : int (L-system generation)
      - kind       : 'root' | 'branch' | 'bridge'
      - family     : 'Y' | 'Z' | None

    No compas.Box is constructed here; Rhino Breps are built in GH.
    """

    DEFAULT_LEN = 250
    DEFAULT_SIZE = 13

    def __init__(
        self,
        axis,
        length=None,
        width=None,
        depth=None,
        parent_frame=None,
        generation=0,
        kind="root",
        family=None,
    ):
        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line.")

        self.axis = axis
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)
        self.generation = int(generation)
        self.kind = str(kind)
        self.family = family  # 'Y', 'Z', or None

        # Orientation: inherit from parent if given, else use axis-based frame
        if parent_frame is not None:
            mid = axis.point_at(0.5)
            x = Vector(parent_frame.xaxis.x, parent_frame.xaxis.y, parent_frame.xaxis.z)
            y = Vector(parent_frame.yaxis.x, parent_frame.yaxis.y, parent_frame.yaxis.z)
            self.frame = Frame(mid, x, y)
        else:
            self.frame = build_frame_from_axis(axis)

        # placeholder, no Box here
        self.geometry = None

    # ------------------------------------------------------------------ #
    # Collision helper                                                   #
    # ------------------------------------------------------------------ #

    def intersects(self, other, clearance=0.0):
        """
        Capsule-like intersection test using axis + max(width, depth).

        This is approximate but sufficient for visual collision debugging.
        """
        if not isinstance(other, Stick):
            return False

        r1 = 0.5 * max(self.width, self.depth) + clearance
        r2 = 0.5 * max(other.width, other.depth) + clearance
        R = r1 + r2

        try:
            d = self.axis.distance_to_line(other.axis)
        except Exception:
            return False

        return d <= R

    def __repr__(self):
        return "Stick(kind={}, fam={}, gen={}, len={:.3f})".format(
            self.kind, self.family, self.generation, self.length
        )
