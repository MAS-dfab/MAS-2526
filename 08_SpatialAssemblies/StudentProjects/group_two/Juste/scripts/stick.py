# stick.py
# r: compas>=2.14.1

from compas.geometry import Line, Vector, Frame, Box


# ------------------------------------------------------------
# Utility: stable perpendicular (used by bridge.py)
# ------------------------------------------------------------
def stable_perp(v):
    """Return a unit vector stably perpendicular to v."""
    if not isinstance(v, Vector):
        v = Vector(*v)

    if v.length < 1e-9:
        return Vector(1.0, 0.0, 0.0)

    v = v.unitized()

    ref = Vector(0.0, 0.0, 1.0)
    if abs(ref.dot(v)) > 0.9:
        ref = Vector(0.0, 1.0, 0.0)

    perp = ref.cross(v)
    if perp.length < 1e-9:
        perp = Vector(1.0, 0.0, 0.0)

    return perp.unitized()


# ------------------------------------------------------------
# Stick class
# ------------------------------------------------------------
class Stick:
    """
    A simple oriented "stick" = box + axis + local frame.

    - axis: compas.geometry.Line (center line of the stick)
    - frame: compas.geometry.Frame (local coordinate system)
    - geometry: compas.geometry.Box (oriented box)
    """

    DEFAULT_LEN = 1.0
    DEFAULT_SIZE = 0.1

    def __init__(self, axis, length=None, width=None, depth=None, parent_frame=None):
        """
        Parameters
        ----------
        axis : Line
            Center line of the stick.
        length : float, optional
        width : float, optional
        depth : float, optional
        parent_frame : Frame, optional
            Frame to inherit orientation from.
        """
        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line.")

        self.axis = axis
        self.length = float(length) if length is not None else self.DEFAULT_LEN
        self.width = float(width) if width is not None else self.DEFAULT_SIZE
        self.depth = float(depth) if depth is not None else self.DEFAULT_SIZE

        # core fix: 3D-aware frame
        self.frame = self._build_frame_from_axis(axis, parent_frame)

        # oriented box: xsize=length, ysize=width, zsize=depth
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

    # ------------------------------------------------------------
    # Frame construction (full safe version)
    # ------------------------------------------------------------
        def _build_frame_from_axis(self, axis, parent_frame=None):

            # --- X direction ---
            x = Vector.from_start_end(axis.start, axis.end)
            if x.length < 1e-9:
                x = Vector(1, 0, 0)
            x = Vector(x.x, x.y, x.z)
            x.unitize()

            # --- Y direction ---
            if parent_frame:
                py = Vector(parent_frame.yaxis.x,
                            parent_frame.yaxis.y,
                            parent_frame.yaxis.z)

                y = py - x * py.dot(x)
                y = Vector(y.x, y.y, y.z)

                if y.length < 1e-6:
                    pz = Vector(parent_frame.zaxis.x,
                                parent_frame.zaxis.y,
                                parent_frame.zaxis.z)
                    y = pz - x * pz.dot(x)
                    y = Vector(y.x, y.y, y.z)

                if y.length < 1e-6:
                    y = Vector(0, 0, 1)

            else:
                y = Vector(0, 0, 1)
                if abs(y.dot(x)) > 0.9:
                    y = Vector(0, 1, 0)

            y.unitize()

            # --- Z direction ---
            z = x.cross(y)
            z = Vector(z.x, z.y, z.z)
            if z.length < 1e-6:
                z = Vector(0, 0, 1)
            z.unitize()

            # --- REPLACE midpoint with point_at() ---
            origin = axis.point_at(0.5)   # SAFE COMPAS POINT
            origin = Vector(origin.x, origin.y, origin.z)  # convert to Vector for Frame constructor

            return Frame(origin, x, y)

    # ------------------------------------------------------------
    # Lightweight collision detection
    # ------------------------------------------------------------
    def intersects(self, other, clearance=0.0):
        """Simple capsule-like collision test between sticks."""
        if not isinstance(other, Stick):
            return False

        r_self = 0.5 * max(self.width, self.depth) + clearance
        r_other = 0.5 * max(other.width, other.depth) + clearance
        r_sum = r_self + r_other

        try:
            d = self.axis.distance_to_line(other.axis)
        except Exception:
            return False

        return d <= r_sum
