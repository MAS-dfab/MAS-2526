from compas.geometry import Line, Vector, Frame, Box


def stable_perp(v):
    """Return a unit vector stably perpendicular to v.

    Used by bridging/branching code to construct local frames.
    """
    if not isinstance(v, Vector):
        v = Vector(*v)

    if v.length < 1e-9:
        return Vector(1.0, 0.0, 0.0)

    v = v.unitized()

    # Choose a reference vector that is not almost parallel to v
    ref = Vector(0.0, 0.0, 1.0)
    if abs(ref.dot(v)) > 0.9:
        ref = Vector(0.0, 1.0, 0.0)

    perp = ref.cross(v)
    if perp.length < 1e-9:
        perp = Vector(1.0, 0.0, 0.0)

    perp.unitize()
    return perp
