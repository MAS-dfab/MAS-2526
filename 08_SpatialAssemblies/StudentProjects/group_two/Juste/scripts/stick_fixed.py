# stick_fixed.py
# Clean Stick class compatible with the new rf_core, branch, and bridge modules.
# No circular imports. Fully self-contained.

from compas.geometry import Point, Vector, Line, Frame, Box


# ======================================================================
# UTILITY — build an orthonormal, stable frame from an axis line
# ======================================================================

def build_frame_from_axis(axis, up_hint=None):
    """
    Build a stable orthonormal Frame from a Line axis.
    axis: compas Line
    up_hint: optional Vector used to disambiguate perpendicular
    """

    start = axis.start
    end = axis.end

    xaxis = Vector.from_start_end(start, end)
    if xaxis.length < 1e-8:
        xaxis = Vector(1, 0, 0)
    xaxis.unitize()

    if up_hint is None:
        up_hint = Vector(0, 0, 1)

    yaxis = up_hint.cross(xaxis)

    # if up_hint is colinear with xaxis → choose fallback
    if yaxis.length < 1e-8:
        up_hint = Vector(0, 1, 0)
        yaxis = up_hint.cross(xaxis)

    if yaxis.length < 1e-8:
        yaxis = Vector(0, 1, 0)

    yaxis.unitize()

    origin = axis.point_at(0.5)

    return Frame(origin, xaxis, yaxis)


# ======================================================================
# MAIN Stick CLASS
# ======================================================================

class Stick:
    """
    Minimal but complete 3D stick geometry container.

    A Stick has:
        - axis (Line)
        - frame (Frame)
        - length (float)
        - width, depth (float)
        - geometry (COMPAS Box)
    """

    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.2

    def __init__(self, axis,
                 length=None, width=None, depth=None,
                 parent_frame=None):

        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line")

        # fundamental geometry
        self.axis = axis
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        # determine orientation
        if parent_frame is not None:
            # inherit orientation; move origin to midpoint of axis
            mid = axis.point_at(0.5)
            x = Vector(*parent_frame.xaxis)
            y = Vector(*parent_frame.yaxis)
            self.frame = Frame(mid, x, y)
        else:
            self.frame = build_frame_from_axis(axis)

        # construct actual box geometry
        # COMPAS Box takes full lengths in X,Y,Z directions of the frame
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

    # ------------------------------------------------------------------
    # COLLISION APPROXIMATION — AABB distance between axes (capsules)
    # ------------------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """
        Approximate intersection test using axis-to-axis distance
        + stick radius (max(width, depth)/2).
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

    # ------------------------------------------------------------------

    def __repr__(self):
        return "Stick(len=%.3f, w=%.3f, d=%.3f)" % (
            self.length, self.width, self.depth
        )
