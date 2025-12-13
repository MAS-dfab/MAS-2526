from compas.geometry import Frame
from compas.geometry import Scale, Transformation


def move_and_scale(assembly):
    """
    Move to the workspace coordinate and scale the assembly to meters.
    
    Args:
        assembly: type compas.datastructures.Assembly
        coordinate: type Point, target coordinate in robot workspace, which is the position for placeing the first stick.
    Returns:
        scaled_assembly: type compas.datastructures.Assembly
    """
    scaled_assembly = assembly.copy()
    factor = 0.001  # 1mm to M

    # X axis point forward to world Z axis
    # to_frame = Frame(coordinate, [0,0,1], [0,1,0])
    
    # Find the center frame at the bottom face of the first stick in each assembly (module)
    first_part = scaled_assembly.find_by_key(key=0)
    from_frame = first_part.attributes["start_frame"]
    # And move the assembly to the target coordinate
    for part in scaled_assembly.parts():
        # O = Transformation.from_frame_to_frame(from_frame, to_frame)
        # part.frame.transform(O)
        # part.attributes["shape"].transform(O)

        # scale to meters
        S = Scale.from_factors([factor, factor, factor], frame=Frame.worldXY())
        part.frame.transform(S)  # target (place) frame
        part.attributes["shape"].scale(factor)
        part.attributes["shape"].frame.scale(factor)


        # part.frame.point.x = -part.frame.point.x  # Invert X axis for UR
        # part.frame.point.y = -part.frame.point.y  # Invert Y axis for UR

        part.attributes["pick_up_geo"].scale(factor)
        part.attributes["pick_up_geo"].frame.scale(factor)
        part.attributes["pick_up_frame"].transform(S)
        # part.attributes["pick_up_frame"].scale(factor)

    return scaled_assembly