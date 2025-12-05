# stick_fixed.py
# Option A: full compas geometry engine.
# Stick owns a compas Box (geometry), axis, frame, and dimensions.

from compas.geometry import Point, Vector, Line, Frame, Box


def build_frame_from_axis(axis, up_hint=None):
    """
    Build a stable orthonormal frame from a Line axis.

    axis    : compas.geometry.Line
    up_hint : optional Vector for resolving perpendicular direction
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
        # if parallel to up_hint, pick another helper
        yaxis = Vector(0, 1, 0).cross(xaxis)

    if yaxis.length < 1e-6:
        yaxis = Vector(0, 1, 0)

    yaxis.unitize()

    # Origin at axis midpoint
    origin = axis.point_at(0.5)

    return Frame(origin, xaxis, yaxis)


class Stick:
    """
    Stick:
      - axis   : compas Line
      - frame  : compas Frame (orientation)
      - length : float (along frame.xaxis)
      - width  : float (along frame.yaxis)
      - depth  : float (along frame.zaxis)
      - geometry : compas Box

    All higher-level logic (branching, bridging, collisions) can rely on this.
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

        # Orientation: inherit from parent if given (and valid), else build from axis.
        if isinstance(parent_frame, Frame):
            mid = axis.point_at(0.5)

            x = Vector(
                parent_frame.xaxis.x,
                parent_frame.xaxis.y,
                parent_frame.xaxis.z,
            )
            y = Vector(
                parent_frame.yaxis.x,
                parent_frame.yaxis.y,
                parent_frame.yaxis.z,
            )

            self.frame = Frame(mid, x, y)
        else:
            self.frame = build_frame_from_axis(axis)

        # ✅ Build compas Box using keyword args (avoids the float/frame confusion).
        self.geometry = Box(
            frame=self.frame,
            xsize=self.length,
            ysize=self.width,
            zsize=self.depth,
        )

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
