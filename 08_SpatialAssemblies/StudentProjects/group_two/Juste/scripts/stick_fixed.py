# stick_fixed.py
# Minimal, robust Stick object for RootFrames / Branching / Bridging.
# No COMPAS Box anymore – only axis, frame, and dimensions.
# Geometry (Brep) will be built in the GH script from these properties.

from compas.geometry import Point, Vector, Line, Frame


class Stick:
    DEFAULT_LEN = 250.0
    DEFAULT_SIZE = 13.0

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        axis : COMPAS Line or RhinoCommon Line-like.
        length/width/depth : float dimensions along frame x/y/z.
        parent_frame : optional COMPAS Frame; if given, we inherit orientation.
        """

        # ---------------------------------------------------------
        # 1. SANITIZE AXIS → COMPAS Line
        # ---------------------------------------------------------
        if isinstance(axis, Line):
            line = axis
        else:
            # Try to interpret RhinoCommon Line-like object
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
        # 2. BUILD A STABLE COMPAS FRAME
        # ---------------------------------------------------------

        # Origin at axis midpoint (nicer visually & for offsets)
        origin = line.point_at(0.5)

        if parent_frame is not None:
            # Inherit parent orientation but move origin to our axis
            xaxis = Vector(
                parent_frame.xaxis.x,
                parent_frame.xaxis.y,
                parent_frame.xaxis.z,
            )
            yaxis = Vector(
                parent_frame.yaxis.x,
                parent_frame.yaxis.y,
                parent_frame.yaxis.z,
            )
            if xaxis.length < 1e-6:
                xaxis = Vector.from_start_end(line.start, line.end)
            xaxis.unitize()
            if yaxis.length < 1e-6:
                yaxis = Vector(0, 1, 0)
            yaxis.unitize()
            self.frame = Frame(origin, xaxis, yaxis)
        else:
            # Derive orientation from axis itself
            xaxis = Vector.from_start_end(line.start, line.end)
            if xaxis.length < 1e-6:
                xaxis = Vector(1, 0, 0)
            xaxis.unitize()

            world_up = Vector(0, 0, 1)
            yaxis = world_up.cross(xaxis)
            if yaxis.length < 1e-6:
                yaxis = Vector(0, 1, 0)
            yaxis.unitize()

            self.frame = Frame(origin, xaxis, yaxis)

        # ---------------------------------------------------------
        # 3. NO COMPAS Box HERE
        # ---------------------------------------------------------
        # Geometry is built later in GH from:
        #   - self.frame
        #   - self.length, self.width, self.depth
        self.geometry = None

        # Optional bookkeeping
        self.children = []
        self.parent_frame = self.frame

    # ---------------------------------------------------------
    # COLLISION HELPERS (AABB-like via axis + radius)
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
        return "Stick(len={:.3f}, w={:.3f}, d={:.3f})".format(
            self.length, self.width, self.depth
        )
