# stick_fixed.py
# Robust Stick class for RootFrames workflow.
# Each stick stores:
#   - axis   : COMPAS Line
#   - frame  : COMPAS Frame (orthonormal)
#   - width/depth/length: dimensions for GH geometry
# No COMPAS Box here; GH builds geometry from the frame.

from compas.geometry import Point, Vector, Line, Frame


def _safe_unit(v, fallback):
    if v.length < 1e-9:
        return fallback.copy()
    v = v.copy()
    v.unitize()
    return v


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        axis : COMPAS Line or RhinoCommon Line-like.
        parent_frame : COMPAS Frame providing orientation.
        """

        # ---------------------------------------------------------
        # 1. SANITIZE AXIS → ALWAYS A COMPAS Line
        # ---------------------------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            try:
                p0 = Point(axis.From.X, axis.From.Y, axis.From.Z)
                p1 = Point(axis.To.X, axis.To.Y, axis.To.Z)
                line = Line(p0, p1)
            except Exception:
                raise ValueError("Stick(): axis argument could not be interpreted as a Line.")

        self.axis = line
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        # ---------------------------------------------------------
        # 2. BUILD LOCAL FRAME
        # ---------------------------------------------------------
        origin = line.point_at(0.5)
        xaxis = Vector.from_start_end(line.start, line.end)
        xaxis = _safe_unit(xaxis, Vector(1, 0, 0))

        if parent_frame is not None:
            # Inherit orientation, override X with axis direction.
            pf_x = parent_frame.xaxis
            pf_y = parent_frame.yaxis
            pf_z = parent_frame.zaxis

            # Maintain right-handedness around axis
            # Compute y as projection of parent y onto plane ⟂ xaxis
            yproj = pf_y - (pf_y.dot(xaxis)) * xaxis
            yaxis = _safe_unit(yproj, Vector(0, 1, 0))
            zaxis = xaxis.cross(yaxis)
            zaxis = _safe_unit(zaxis, Vector(0, 0, 1))
            self.frame = Frame(origin, xaxis, yaxis)

        else:
            # Independent stick — build clean frame
            # Try using world-up only if safe
            world_up = Vector(0, 0, 1)
            yaxis = world_up.cross(xaxis)
            yaxis = _safe_unit(yaxis, Vector(0, 1, 0))

            self.frame = Frame(origin, xaxis, yaxis)

        self.children = []
        self.parent_frame = self.frame

    # ---------------------------------------------------------
    # COLLISION HELPERS
    # ---------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """Capsule-like collision based on distance between axes."""
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
        return "Stick(len={:.1f}, w={:.1f}, d={:.1f})".format(
            self.length, self.width, self.depth
        )
