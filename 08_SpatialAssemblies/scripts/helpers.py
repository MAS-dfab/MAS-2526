from compas.geometry import intersection_line_plane, Plane, Translation, distance_point_point, Frame, Scale
from compas.geometry import Point, bounding_box
import math

def sort_sticks_by_z(sticks):
    """
    Sorts a list of sticks by their z-coordinate.

    Args:
        sticks (list): A list of stick objects.

    Returns:
        list: A list of stick objects sorted by their z-coordinate.
    """
    return sorted(sticks, key=lambda x: min(x.axis.start.z, x.axis.end.z))


def scale_and_move_to_point(assembly, center):


    scaled_assembly = assembly.copy()
    factor = 0.001 # 1mm to M

    #scale to 1mm
    for part in scaled_assembly.parts():
        S = Scale.from_factors([factor, factor, factor], frame=Frame.worldXY())
        part.frame.transform(S)
        part.attributes["shape"].scale(factor)
        part.attributes["shape"].frame.scale(factor)


    points = [p for part in scaled_assembly.parts() for p in part.attributes["shape"].vertices]
    bbox = bounding_box(points)
    cur_center = Point(0,0,bbox[0][2])

    T = Translation.from_vector(center-cur_center)

    for part in scaled_assembly.parts():
        part.frame.transform(T)
        part.attributes["shape"].transform(T)

    return scaled_assembly


def generate_default_tolerances(joints):
    DEFAULT_TOLERANCE_METERS = 0.001
    DEFAULT_TOLERANCE_RADIANS = math.radians(0.1)

    return [DEFAULT_TOLERANCE_METERS if j.is_scalable() else DEFAULT_TOLERANCE_RADIANS for j in joints]


APPROACH_DISTANCE = 0.1  # 10 cm
    
def calculate_pick_trajectory(pickup_frame, robot, start_config, group = "manipulator"):
    """
    Calculate the pick trajectory for a given pick frame.
    """
    pick_frame = pickup_frame.copy()
    pick_frame.point.x = -pick_frame.point.x  # Invert X axis for UR
    pick_frame.point.y = -pick_frame.point.y  # Invert Y axis for UR
    # Find IK solution for pick frame
    approach_pick_frame = pick_frame.copy()
    approach_pick_frame.translate(
        APPROACH_DISTANCE * -approach_pick_frame.zaxis
    )

    # Generate cartesian trajectory from pick to approach pick frame
    max_step = 0.1
    trajectory = robot.plan_cartesian_motion(
        [approach_pick_frame, pick_frame],
        start_configuration=start_config,
        group=group,
        options=dict(
            max_step=max_step,
        ),
    )

    # Check if trajectory is complete
    if trajectory.fraction < 1:
        raise Exception(
            "Incomplete cartesian trajectory found. Only {:.1f}% of the trajectory could be planned".format(
                trajectory.fraction * 100
            )
        )

    # Return trajectory, pick configuration and approach pick configuration
    return trajectory, trajectory.points[-1], trajectory.points[0]


def calculate_place_trajectories(robot, current_config,  placement_frame, group="manipulator"):
    """
    Calculates the  place trajectory (to place_frame), 
    and return trajectory (back to safe_config) for a part.
    """
    place_frame = placement_frame.copy()

    start_config_for_place = current_config
    goal_constraints_place = robot.constraints_from_frame(
        place_frame,
        tolerance_position=0.0001,
        tolerances_axes=[0.0001, 0.0001, 0.0001],
        use_attached_tool_frame=True,
        group=group or robot.main_group_name,
    )
    place_trajectory = robot.plan_motion(
        goal_constraints_place,
        start_configuration=start_config_for_place,
        group=group or robot.main_group_name,
        options=dict(
            planner_id="RRTConnect",
            avoid_collisions=True,
        ),
    )
    print("Planned place trajectory.")

    # Go to exit frame (safe distance above place frame)

    exit_frame = place_frame.copy()
    exit_frame.translate(APPROACH_DISTANCE * -exit_frame.zaxis)
    
    exit_trajectory = robot.plan_cartesian_motion(
        [place_frame, exit_frame],
        start_configuration=place_trajectory.points[-1],
        group=group or robot.main_group_name,
        options=dict(
            max_step=0.01,
        ),  
    )
    print("Planned safe trajectory.")

    # Go to safe configuration (home)
    safe_config = start_config_for_place
    safe_constraints = robot.constraints_from_configuration(
        safe_config,
        tolerances_above = [0.0001]*6,
        tolerances_below = [0.0001]*6,
    )
    return_trajectory = robot.plan_motion(
        safe_constraints,
        start_configuration=exit_trajectory.points[-1],
        group=group or robot.main_group_name,
        options=dict(
            planner_id="RRTConnect",
            avoid_collisions=True,
        ),
    )
    print("Planned return trajectory.")
    if place_trajectory.fraction < 1:
        raise Exception(
            "Incomplete place trajectory found. Only {:.1f}% of the trajectory could be planned".format(
                place_trajectory.fraction * 100
            )
        )
    if exit_trajectory.fraction < 1:
        raise Exception(
            "Incomplete exit trajectory found. Only {:.1f}% of the trajectory could be planned".format(
                exit_trajectory.fraction * 100
            )
        )
    if return_trajectory.fraction < 1:
        raise Exception(
            "Incomplete return trajectory found. Only {:.1f}% of the trajectory could be planned".format(
                return_trajectory.fraction * 100
            )
        )
    
    joined_exit_trajectory = exit_trajectory.copy()
    joined_exit_trajectory.points.extend(return_trajectory.points)
    return place_trajectory, joined_exit_trajectory