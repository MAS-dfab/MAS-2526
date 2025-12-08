# stick_fixed.py
# Robust Stick class for RootFrames workflow.
# Stores:
#   - axis   : COMPAS Line
#   - frame  : COMPAS Frame (orthonormal)
#   - length / width / depth
# No geometry here; GH builds Breps from frame + dimensions.

from compas.geometry import Point, Vector, Line, Frame


def _safe_unit(vec, fallback):
    """Return a unit copy of vec, or a unit copy of fallback if degenerate."""
    v = vec.copy()
    if v.length < 1e-9:
        v = fallback.copy()
    v.unitize()
    return v


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        axis : COMPAS Line or RhinoCommon Line-like.
        parent_frame : optional COMPAS Frame for orientation inheritance.
        """

        # ---------------------------------------------------------
        # 1. AXIS → COMPAS Line
        # ---------------------------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            # Try RhinoCommon-style .From / .To
            try:
                p0 = Point(axis.From.X, axis.From.Y, axis.From.Z)
                p1 = Point(axis.To.X, axis.To.Y, axis.To.Z)
                line = Line(p0, p1)
            except Exception:
                raise ValueError("Stick(): axis is not a valid Line-like object.")

        self.axis = line
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width = float(width or Stick.DEFAULT_SIZE)
        self.depth = float(depth or Stick.DEFAULT_SIZE)

        # ---------------------------------------------------------
        # 2. LOCAL FRAME
        # ---------------------------------------------------------
        origin = line.point_at(0.5)
        xaxis = Vector.from_start_end(line.start, line.end)
        xaxis = _safe_unit(xaxis, Vector(1, 0, 0))

        if parent_frame is not None:
            # Inherit parent orientation; project its y-axis into plane ⟂ xaxis
            pf_y = parent_frame.yaxis
            yproj = pf_y - (pf_y.dot(xaxis)) * xaxis
            yaxis = _safe_unit(yproj, Vector(0, 1, 0))
            self.frame = Frame(origin, xaxis, yaxis)
        else:
            # Free-standing stick
            world_up = Vector(0, 0, 1)
            yaxis = world_up.cross(xaxis)
            yaxis = _safe_unit(yaxis, Vector(0, 1, 0))
            self.frame = Frame(origin, xaxis, yaxis)

        # ---------------------------------------------------------
        # 3. BOOKKEEPING / TAGS (for colors & debugging)
        # ---------------------------------------------------------
        self.children = []
        self.parent_frame = self.frame

        self.is_root = False      # True for root sticks
        self.family = None        # "Y", "Z", "BRIDGE" or None
        self.is_bridge = False
        self.collided = False     # set in RootFrames.detect_collisions

    # ---------------------------------------------------------
    # COLLISION HELPER
    # ---------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """Cheap capsule-like collision: distance between axes vs radii."""
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
        return "Stick(len={:.1f}, w={:.1f}, d={:.1f}, fam={})".format(
            self.length, self.width, self.depth, self.family
        )
