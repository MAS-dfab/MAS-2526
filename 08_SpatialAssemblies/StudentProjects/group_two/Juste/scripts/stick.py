# stick.py
# r: compas>=2.14.1

from compas.geometry import Line, Vector, Frame, Box


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
            Box length along local X (axis direction).
        width : float, optional
            Box width along local Y.
        depth : float, optional
            Box depth along local Z.
        parent_frame : Frame, optional
            Frame to inherit orientation from. If provided, the stick's
            frame will keep its Y/Z directions as close as possible to
            the parent frame while aligning X with the axis.
        """
        if not isinstance(axis, Line):
            raise TypeError("Stick axis must be a compas.geometry.Line.")

        self.axis = axis
        self.length = float(length) if length is not None else self.DEFAULT_LEN
        self.width = float(width) if width is not None else self.DEFAULT_SIZE
        self.depth = float(depth) if depth is not None else self.DEFAULT_SIZE

        # build local frame (3D-aware)
        self.frame = self._build_frame_from_axis(axis, parent_frame)

        # oriented box geometry: xsize = length, ysize = width, zsize = depth
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

    # ----------------------------------------------------------------------
    # Frame construction
    # ----------------------------------------------------------------------

    def _build_frame_from_axis(self, axis, parent_frame=None):
        """
        Build a local Frame whose X-axis follows the axis direction and whose
        Y/Z axes inherit orientation from `parent_frame` when possible.

        This is the key to avoiding "flattening": each stick keeps the
        parent's roll/twist instead of re-orthogonalising in world coordinates.
        """
        # X direction = axis direction
        x = Vector.from_start_end(axis.start, axis.end)
        if x.length < 1e-9:
            x = Vector(1.0, 0.0, 0.0)
        x.unitize()

        if parent_frame is not None:
            # Try to keep parent Y as much as possible
            py = parent_frame.yaxis
            # Remove any component along x (project onto plane ⟂ x)
            y = py - x * py.dot(x)

            if y.length < 1e-6:
                # If parent Y was almost parallel to x, try parent Z
                pz = parent_frame.zaxis
                y = pz - x * pz.dot(x)

            if y.length < 1e-6:
                # Fallback if parent frame is degenerate
                y = Vector(0.0, 0.0, 1.0)
        else:
            # No parent: pick a stable world reference
            y = Vector(0.0, 0.0, 1.0)
            if abs(y.dot(x)) > 0.9:
                y = Vector(0.0, 1.0, 0.0)

        y.unitize()
        z = x.cross(y)
        if z.length < 1e-6:
            z = Vector(0.0, 0.0, 1.0)
        z.unitize()

        # Frame origin at axis midpoint
        origin = axis.midpoint
        return Frame(origin, x, y)

    # ----------------------------------------------------------------------
    # Collision helper (very simple placeholder)
    # ----------------------------------------------------------------------

    def intersects(self, other, clearance=0.0):
        """
        Very simple collision heuristic between two sticks.

        NOTE:
            This is intentionally lightweight. It treats each stick as a
            capsule-like volume: if the closest distance between the two
            axes is smaller than a radius sum, we flag intersection.

            You can replace this with a more robust Box-vs-Box test later.
        """
        if not isinstance(other, Stick):
            return False

        # approximate radius as half of max(width, depth)
        r_self = 0.5 * max(self.width, self.depth) + clearance
        r_other = 0.5 * max(other.width, other.depth) + clearance
        r_sum = r_self + r_other

        # closest distance between two segments (axes)
        d = self.axis.distance_to_line(other.axis)  # compas method
        return d <= r_sum
