# stick_fixed.py
# COMPAS-only stick: axis + frame + dimensions + simple BoxGeom container

from compas.geometry import Point, Vector, Line, Frame


class BoxGeom(object):
    """Lightweight box representation in COMPAS coordinates."""
    def __init__(self, frame, xsize, ysize, zsize):
        if not isinstance(frame, Frame):
            raise TypeError("BoxGeom.frame must be a compas Frame.")
        self.frame = frame
        self.xsize = float(xsize)
        self.ysize = float(ysize)
        self.zsize = float(zsize)

    def __repr__(self):
        return "BoxGeom(x={:.3f}, y={:.3f}, z={:.3f})".format(
            self.xsize, self.ysize, self.zsize
        )


def build_frame_from_axis(axis, up_hint=None):
    """Build a stable orthonormal frame from a Line axis."""
    start = axis.start
    end = axis.end

    x = Vector.from_start_end(start, end)
    if x.length < 1e-6:
        x = Vector(1, 0, 0)
    x.unitize()

    if up_hint is None:
        up_hint = Vector(0, 0, 1)

    y = up_hint.cross(x)
    if y.length < 1e-6:
        y = Vector(0, 1, 0)
    y.unitize()

    origin = axis.point_at(0.5)
    return Frame(origin, x, y)


class Stick(object):
    """
    Stick:
      - axis   : compas Line
      - frame  : compas Frame
      - length : float (along frame.xaxis)
      - width  : float (along frame.yaxis)
      - depth  : float (along frame.zaxis)
      - geometry : BoxGeom (COMPAS-level box)
    """

    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.2

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas Line.")

        self.axis = axis
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        # Orientation: inherit parent frame if valid, else build from axis
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

        # Pure COMPAS box geometry container (no compas Shape/Box)
        self.geometry = BoxGeom(
            frame=self.frame,
            xsize=self.length,
            ysize=self.width,
            zsize=self.depth,
        )

    # Collision helper
    def intersects(self, other, clearance=0.0):
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
        return "Stick(len={:.3f}, w={:.3f}, d={:.3f})".format(
            self.length, self.width, self.depth
        )
