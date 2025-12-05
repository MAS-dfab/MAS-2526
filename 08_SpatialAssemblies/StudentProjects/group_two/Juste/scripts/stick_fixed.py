# stick_fixed.py
# Final stable version — NO compas Box, no Shape.frame usage.

from compas.geometry import Point, Vector, Line, Frame


def build_frame_from_axis(axis, up_hint=None):
    """
    Build a stable orthonormal frame from a Line axis.

    axis    : compas.geometry.Line
    up_hint : optional Vector used to resolve perpendicular direction
    """
    start = axis.start
    end   = axis.end

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
        # If parallel to up_hint, choose a new vector
        yaxis = Vector(0, 1, 0).cross(xaxis)

    if yaxis.length < 1e-6:
        yaxis = Vector(0, 1, 0)

    yaxis.unitize()

    # Origin at axis midpoint
    origin = axis.point_at(0.5)

    return Frame(origin, xaxis, yaxis)


class Stick:
    """
    Minimal stick representation (no compas Box):

      - axis   : compas Line
      - frame  : compas Frame (orientation)
      - length : float (along frame.xaxis)
      - width  : float (along frame.yaxis)
      - depth  : float (along frame.zaxis)

    NOTE:
        No compas Box() object is created here — GH builds its own Rhino Brep.
    """

    DEFAULT_LEN  = 1.0
    DEFAULT_SIZE = 0.2

    def __init__(self, axis: Line, length=None, width=None, depth=None, parent_frame=None):

        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line.")

        self.axis   = axis
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width  = float(width  or Stick.DEFAULT_SIZE)
        self.depth  = float(depth  or Stick.DEFAULT_SIZE)

        # Orientation: inherit from parent if given, else build frame from axis
        if parent_frame is not None:
            mid = axis.point_at(0.5)

            # Copy parent orientation safely
            x = Vector(parent_frame.xaxis.x,
                       parent_frame.xaxis.y,
                       parent_frame.xaxis.z)
            y = Vector(parent_frame.yaxis.x,
                       parent_frame.yaxis.y,
                       parent_frame.yaxis.z)

            self.frame = Frame(mid, x, y)

        else:
            self.frame = build_frame_from_axis(axis)

        # IMPORTANT:
        # DO NOT construct a compas Box here — Box(Frame) triggers Shape.frame setter,
        # which causes the 'float is not subscriptable' crash under Rhinocode.
        self.geometry = None

    # ----------------------------------------------------------------------
    # Collision helper
    # ----------------------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """Capsule-like intersection test using axis + width/depth."""

        if not isinstance(other, Stick):
            return False

        r1 = 0.5 * max(self.width, self.depth) + clearance
        r2 = 0.5 * max(other.width, other.depth) + clearance
        R  = r1 + r2

        try:
            d = self.axis.distance_to_line(other.axis)
        except Exception:
            return False

        return d <= R

    def __repr__(self):
        return "Stick(len={}, w={}, d={})".format(self.length, self.width, self.depth)
