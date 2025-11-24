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
        part.transform(S)
        part.frame.transform(S)

    points = [p for part in scaled_assembly.parts() for p in part.shape.vertices]
    bbox = bounding_box(points)
    cur_center = Point(0,0,bbox[0][2])

    T = Translation.from_vector(center-cur_center)

    for part in scaled_assembly.parts():
        part.transform(T)
        part.frame.transform(T)
        part.attributes["midpoint"].transform(T)

    return scaled_assembly


def generate_default_tolerances(joints):
    DEFAULT_TOLERANCE_METERS = 0.001
    DEFAULT_TOLERANCE_RADIANS = math.radians(0.1)

    return [DEFAULT_TOLERANCE_METERS if j.is_scalable() else DEFAULT_TOLERANCE_RADIANS for j in joints]


APPROACH_DISTANCE = 0.1  # 10 cm
    
def calculate_pick_trajectory(pick_frame, robot, start_config, group = "manipulator"):
    """
    Calculate the pick trajectory for a given pick frame.
    Args:
        pick_frame (compas.geometry.Frame): The pick frame.
        robot (compas_fab.robots.Robot): The robot instance.
    Returns:
        tuple: The pick trajectory, the pick configuration and the approach pick configuration.
    """
    # Find IK solution for pick frame
    approach_pick_frame = pick_frame.copy()
    approach_pick_frame.translate(
        -APPROACH_DISTANCE * approach_pick_frame.zaxis
    )

    # Generate cartesian trajectory from pick to approach pick frame
    max_step = 0.01
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


def calculate_place_trajectories(robot, current_config, safe_frame, place_frame, group=None):
    """
    Calculates the safe trajectory (to safe_frame), place trajectory (to place_frame), 
    and return trajectory (back to safe_frame) for a part.
    
    Args:
        robot: compas_fab.robots.Robot instance.
        current_config: Starting Configuration.
        safe_frame: Frame to move to before/after placing.
        place_frame: Frame where the part should be placed.
        group: (optional) robot group name.
    
    Returns:
        dict with keys: 'safe_trajectory', 'place_trajectory', 'return_trajectory'
    """
    # Move to safe_frame
    goal_constraints_safe = robot.constraints_from_frame(
        safe_frame,
        tolerance_position=0.001,
        tolerances_axes=[0.001, 0.001, 0.001],
        use_attached_tool_frame=True,
        group=group or robot.main_group_name,
    )
    safe_trajectory = robot.plan_motion(
        goal_constraints_safe,
        start_configuration=current_config,
        group=group or robot.main_group_name,
        options=dict(
            planner_id="RRTConnect",
            avoid_collisions=True,
        ),
    )

    # Move from safe_frame to place_frame
    goal_constraints_place = robot.constraints_from_frame(
        place_frame,
        tolerance_position=0.001,
        tolerances_axes=[0.001, 0.001, 0.001],
        use_attached_tool_frame=True,
        group=group or robot.main_group_name,
    )
    place_trajectory = robot.plan_cartesian_motion(
        [safe_frame, place_frame], 
        start_configuration=safe_trajectory.points[-1],
        group=group or robot.main_group_name,
        options=dict(
            max_step=0.01,
        ),
    )   

    # Return from place_frame to safe_frame
    return_trajectory = robot.plan_motion(
        goal_constraints_safe,
        start_configuration=place_trajectory.points[-1],
        group=group or robot.main_group_name,
        options=dict(
            planner_id="RRTConnect",
            avoid_collisions=True,
        ),
    )

    return {
        "safe_trajectory": safe_trajectory,
        "place_trajectory": place_trajectory,
        "return_trajectory": return_trajectory
    }