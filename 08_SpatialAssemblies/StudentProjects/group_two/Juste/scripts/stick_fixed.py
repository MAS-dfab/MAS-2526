# stick_fixed.py
# A fully corrected 3D Stick class producing valid compas Frames and Boxes

from compas.geometry import Point, Vector, Line, Frame, Box


# ------------------------------------------------------------------------------
# Frame builder (stable, robust, works for any axis direction)
# ------------------------------------------------------------------------------

def build_frame_from_axis(axis, up_hint=Vector(0, 0, 1)):
    """
    Build a stable orthonormal frame from a Line axis.
    axis    : compas.geometry.Line
    up_hint : Vector used to resolve perpendicular direction
    """

    start = axis.start
    end   = axis.end

    # Primary axis direction
    xaxis = Vector.from_start_end(start, end)
    if xaxis.length < 1e-6:
        xaxis = Vector(1, 0, 0)
    xaxis.unitize()

    # Compute a stable perpendicular using up_hint
    yaxis = up_hint.cross(xaxis)
    if yaxis.length < 1e-6:
        # If parallel to up_hint, choose another vector
        yaxis = Vector(0, 1, 0).cross(xaxis)
    yaxis.unitize()

    # zaxis implied by right-hand orientation
    zaxis = xaxis.cross(yaxis)
    if zaxis.length < 1e-6:
        zaxis = Vector(0, 0, 1)
    zaxis.unitize()

    return Frame(start, xaxis, yaxis)


# ------------------------------------------------------------------------------
# Stick class
# ------------------------------------------------------------------------------

class Stick:
    DEFAULT_LEN  = 1.0
    DEFAULT_SIZE = 0.2

    def __init__(self, axis: Line, length=None, width=None, depth=None, parent_frame=None):
        """
        axis         : compas.geometry.Line
        parent_frame : optional Frame to enforce orientation continuity
        """

        # Store axis
        self.axis   = axis

        # Dimensions
        self.length = float(length or Stick.DEFAULT_LEN)
        self.width  = float(width  or Stick.DEFAULT_SIZE)
        self.depth  = float(depth  or Stick.DEFAULT_SIZE)

        # ------------------------------------------------------------------
        # Compute orientation frame
        # ------------------------------------------------------------------
        if parent_frame:
            # Inherit parent's orthonormal frame orientation
            self.frame = parent_frame.copy()

            # Update origin but keep orientation
            origin_shift = axis.midpoint
            self.frame.point = origin_shift

        else:
            # Full 3D reconstruction from axis (correct, stable)
            self.frame = build_frame_from_axis(axis)

        # ------------------------------------------------------------------
        # Build solid geometry
        # COMPAS Box signature:
        #     Box(frame, xsize, ysize, zsize)
        # ------------------------------------------------------------------
        self.geometry = Box(self.frame, self.length, self.width, self.depth)

    def __repr__(self):
        return "Stick(len={}, w={}, d={})".format(self.length, self.width, self.depth)
