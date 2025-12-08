# stick.py
# COMPAS-native Stick with stable 3D frame + Box geometry
# Offset rule: ONE-SIDED (parent.width or parent.depth only)

from compas.geometry import Point, Vector, Line, Frame, Box


# ------------------------------------------------------------
# Helper — build stable orthonormal frame from axis
# ------------------------------------------------------------
def build_frame_from_axis(axis: Line) -> Frame:
    start = axis.start
    end   = axis.end

    # X-axis
    x = Vector.from_start_end(start, end)
    if x.length < 1e-6:
        x = Vector(1, 0, 0)
    x.unitize()

    # Y-axis (perpendicular)
    up = Vector(0, 0, 1)
    y = up.cross(x)

    if y.length < 1e-6:
        # fallback if axis almost parallel to Z
        y = Vector(0, 1, 0).cross(x)

    y.unitize()

    # Origin = center of axis
    origin = axis.point_at(0.5)

    return Frame(origin, x, y)


# ------------------------------------------------------------
# Main Stick Class
# ------------------------------------------------------------
class Stick:
    LENGTH  = 250.0
    SIZE = 13

    def __init__(
        self,
        axis: Line,
        length=None,
        width=None,
        depth=None,
        parent_frame: Frame = None,
    ):
        if not isinstance(axis, Line):
            raise TypeError("Stick requires axis = compas.geometry.Line")

        self.axis   = axis
        self.length = float(length or Stick.LENGTH)
        self.width  = float(width  or Stick.SIZE)
        self.depth  = float(depth  or Stick.SIZE)

        # --------------------------------------------------------
        # Frame selection (Option A – default COMPAS: Frame(x, y))
        # --------------------------------------------------------
        if parent_frame is None:
            # Build frame from axis
            base_frame = build_frame_from_axis(axis)
        else:
            # Inherit orientation from parent frame — reposition origin
            mid = axis.point_at(0.5)
            x = Vector(parent_frame.xaxis.x,
                       parent_frame.xaxis.y,
                       parent_frame.xaxis.z)
            y = Vector(parent_frame.yaxis.x,
                       parent_frame.yaxis.y,
                       parent_frame.yaxis.z)
            base_frame = Frame(mid, x, y)

        self.frame = base_frame

        # --------------------------------------------------------
        # Build COMPAS Box geometry (used for visualization + collision)
        # Box dimensions correspond to local axes:
        #   x-axis → length
        #   y-axis → width
        #   z-axis → depth
        # --------------------------------------------------------
        L = self.length
        W = self.width
        D = self.depth
        self.geometry = Box(self.frame, L, W, D)

    # ------------------------------------------------------------
    # Capsule-like collision test
    # ------------------------------------------------------------
    def intersects(self, other, clearance=0.0):
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

    # Debug print
    def __repr__(self):
        return "Stick(len={}, w={}, d={})".format(
            self.length, self.width, self.depth
        )
