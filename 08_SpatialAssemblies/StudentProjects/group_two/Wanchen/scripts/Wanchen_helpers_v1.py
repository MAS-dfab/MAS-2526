from compas.geometry import Frame
from compas.geometry import Scale, Transformation


def move_and_scale(assembly):
    """
    Move to the workspace coordinate and scale the assembly to meters.
    
    Args:
        assembly: type compas.datastructures.Assembly
    Returns:
        scaled_assembly: type compas.datastructures.Assembly
    """
    scaled_assembly = assembly.copy()
    factor = 0.001  # 1mm to M

    # Find the center frame at the bottom face of the first stick in each assembly (module)
    first_part = scaled_assembly.find_by_key(key=0)
    if first_part is None:
        raise Exception("First part with key=0 not found in assembly")
    
    from_frame = first_part.attributes.get("start_frame", None)
    if from_frame is None:
        raise Exception("start_frame not found in first part attributes")
    
    print(f"Starting move_and_scale with factor={factor}")
    
    # And move the assembly to the target coordinate
    for part in scaled_assembly.parts():
        try:
            # scale to meters
            S = Scale.from_factors([factor, factor, factor], frame=Frame.worldXY())
            part.frame.transform(S)
            part.attributes["shape"].scale(factor)
            part.attributes["shape"].frame.scale(factor)

            # 安全地处理可选的 attributes
            if "pick_up_geo" in part.attributes and part.attributes["pick_up_geo"] is not None:
                part.attributes["pick_up_geo"].scale(factor)
                part.attributes["pick_up_geo"].frame.scale(factor)
            
            if "pick_up_frame" in part.attributes and part.attributes["pick_up_frame"] is not None:
                part.attributes["pick_up_frame"].transform(S)
                
        except Exception as e:
            print(f"Error processing part {part.key}: {e}")
            raise

    print("move_and_scale completed successfully")
    return scaled_assembly