# stick_fixed.py
# Final stable version — NO compas Box, no Shape.frame usage.

from compas.geometry import Point, Vector, Line, Frame


def build_frame_from_axis(axis, up_hint=None):
    """
    Build a stable orthonormal frame from a Line axis.

    axis    : compas.geometry.Line
    up_hint : optional Vector used to resolve perpendicular direction
    """
    start = axis.start
    end   = axis.end

    # X-axis along the stick
    xaxis = Vector.from_start_end(start, end)
    if xaxis.length < 1e-6:
        xaxis = Vector(1, 0, 0)
    xaxis.unitize()

    #
