from compas.datastructures import Assembly, Mesh
from compas.geometry import Frame, Point, Translation, Transformation
from compas_fab.backends import RosClient
from compas_fab.robots import Tool, PlanningScene, CollisionMesh
import os, sys
import math

import traceback

import compas
from compas.data import json_load
from compas_robots import Configuration

compas.PRECISION = "12f"

# Constants
APPROACH_DISTANCE = 0.015  # Meters
group = None
start_configuration = None

# Specify the path to the assembly file and the path to the output file
path = ghenv.Component.OnPingDocument().FilePath
#parentdir = os.path.dirname(path)
parentdir = r"D:\0000_ETH_DFAB\Github\MAS-2526\08_SpatialAssemblies\StudentProjects\group_two\Wanchen"
dire = os.path.join(parentdir, "scripts")
sys.path.append(dire)
json_path = os.path.join(parentdir, "data", "stick_assembly.json")
write_path = os.path.join(parentdir, "data", "stick_assembly_with_trajectories.json")

# Import helpers
from compas_rhino import unload_modules
unload_modules("helpers")
unload_modules("J3RRY_helpers_v1")
r_path = r"D:\0000_ETH_DFAB\Github\MAS-2526\08_SpatialAssemblies\StudentProjects\group_two\J3RRY_Sticks_^o^\robot_related\scripts"
sys.path.append(r_path)

json_path = os.path.join(parentdir, "data", "stick_assembly.json")
write_path = os.path.join(parentdir, "data", "stick_assembly_with_trajectories.json")

# Import helpers
from compas_rhino import unload_modules
unload_modules("helpers")
unload_modules("J3RRY_helpers_v1")

from compas_rhino.conversions import mesh_to_compas, point_to_compas, frame_to_rhino_plane
from helpers import scale_and_move_to_point, generate_default_tolerances, calculate_pick_trajectory, calculate_place_trajectories
from scriptcontext import sticky
from J3RRY_helpers_v1 import move_and_scale


if "last_status" not in sticky or sticky["last_status"] is None:
    sticky["last_status"] = "Idle"

if "geometry" not in sticky or sticky["geometry"] is None:
    pass

SAFE_CONFIG = Configuration.from_revolute_values([math.radians(68.80), 
                                        math.radians(-70.55), 
                                        math.radians(-116.42), 
                                        math.radians(-83.80), 
                                        math.radians(98.73), 
                                        math.radians(56.93)])

if 'load' in globals() and load:
    sticky["assembly"] = None
    sticky["geometry"] = None
    sticky["results"] = {}
    
    scene.reset()
    tcm = CollisionMesh(mesh_to_compas(env), id = "table")
    scene.add_collision_mesh(tcm)
    assembly = json_load(json_path)
    scaled_assembly = scale_and_move_to_point(assembly, Point(0,0,0))

    sticky["assembly"] = scaled_assembly
    sticky["last_status"] = "Assembly loaded"
else:
    scaled_assembly = sticky.get("assembly", None)  
    if scaled_assembly:
        part = scaled_assembly.find_by_key(index)  
        all_sticks = [p.attributes.get("shape", None) for p in scaled_assembly.parts()]  
        plns_out = [frame_to_rhino_plane(p.frame) for p in scaled_assembly.parts()]  
        pick_plns_out = [frame_to_rhino_plane(p.attributes.get("pick_up_frame")) for p in scaled_assembly.parts()]  
        pick_up_geos = [p.attributes.get("pick_up_geo", None) for p in scaled_assembly.parts()]  
        sticky["geometry"] = part.attributes.get("shape", None)

# If it failed or strange, just compute more times!

if 'compute' in globals() and compute and 'index' in globals():
    results = sticky.get("results", {})
    results.pop(str(index), None)
    try:
        part = scaled_assembly.find_by_key(index)

        # define pick_frame and safe_frame
        pick_frame = part.attributes.get("pick_up_frame")
        safe_frame = Frame(
            Point(pick_frame.point.x, pick_frame.point.y, pick_frame.point.z),
            pick_frame.xaxis,
            pick_frame.yaxis
        )
        safe_frame.translate(-0.2 * safe_frame.zaxis)

        # place_frame construct
        place_frame = part.frame

        # define place_safe_frame
        place_safe_frame = Frame(
            Point(place_frame.point.x, place_frame.point.y, place_frame.point.z),
            place_frame.xaxis,
            place_frame.yaxis
        )
        place_safe_frame.translate(-0.2 * place_safe_frame.zaxis)

        tolerances = generate_default_tolerances(robot.get_configurable_joints(robot.main_group_name))

        # ---------- Pick trajectory ----------
        pick_traj, pick_config, approach_config = calculate_pick_trajectory(
            pick_frame, robot, start_config=SAFE_CONFIG
        )

        # ---------- Pick -> Safe (RRTConnect) ----------
        return_to_safe = robot.plan_motion(
            robot.constraints_from_frame(
                safe_frame,
                tolerance_position=0.05,
                tolerances_axes=[0.5, 0.5, 0.5],
                use_attached_tool_frame=True,
                group=robot.main_group_name,
            ),
            start_configuration=pick_config,
            group=robot.main_group_name,
            options=dict(
                planner_id="RRTConnect",
                avoid_collisions=True,
                timeout=380.0,  # Please don't reduce this timeout,otherwise it will failed more frequently
            ),
        )
        if not return_to_safe:
            raise Exception("Return to safe (plan_motion) failed")

        #---------- Safe -> Place_safe (RRTConnect) ----------
        to_place_safe = robot.plan_motion(
            robot.constraints_from_frame(
                place_safe_frame,
                tolerance_position=0.05,
                tolerances_axes=[0.5, 0.5, 0.5],
                use_attached_tool_frame=True,
                group=robot.main_group_name,
            ),
            start_configuration=return_to_safe.points[-1],
            group=robot.main_group_name,
            options=dict(
                planner_id="RRTConnect",
                avoid_collisions=True,
                timeout=380.0,# Please don't reduce this timeout,otherwise it will failed more frequently
            ),
        )
        

        # ---------- Place_safe -> Place_frame (Cartesian) ----------
        to_place = robot.plan_cartesian_motion(
            [place_safe_frame, place_frame],
            start_configuration=to_place_safe.points[-1],
            group=robot.main_group_name,
            options=dict(max_step=0.05),
        )
        

        # ---------- Place_frame -> Place_safe (Cartesian) ----------
        back_to_place_safe = robot.plan_cartesian_motion(
            [place_frame, place_safe_frame],
            start_configuration=to_place.points[-1],
            group=robot.main_group_name,
            options=dict(max_step=0.05),
        )
        
        # ---------- Place_safe -> Place_frame (Cartesian) ----------
        to_place_again = robot.plan_cartesian_motion(
            [place_safe_frame, place_frame],
            start_configuration=back_to_place_safe.points[-1],
            group=robot.main_group_name,
            options=dict(max_step=0.05),
        )
        
        # ---------- Place_frame -> Place_safe (Cartesian) ----------
        exit_from_place = robot.plan_cartesian_motion(
            [place_frame, place_safe_frame],
            start_configuration=to_place_again.points[-1],
            group=robot.main_group_name,
            options=dict(max_step=0.05),
        )

        # ---------- Place_safe -> Safe (RRTConnect) ----------
        back_to_safe = robot.plan_motion(
            robot.constraints_from_frame(
                safe_frame,
                tolerance_position=0.05,
                tolerances_axes=[0.5, 0.5, 0.5],
                use_attached_tool_frame=True,
                group=robot.main_group_name,
            ),
            start_configuration=exit_from_place.points[-1],
            group=robot.main_group_name,
            options=dict(
                planner_id="RRTConnect",
                avoid_collisions=True,
                timeout=380.0,# Please don't reduce this timeout,otherwise it will failed more frequently
            ),
        )

        # ---------- Combine all trajectory----------
        joined_trajectory = pick_traj.copy()
        joined_trajectory.points.extend(return_to_safe.points[1:])
        joined_trajectory.points.extend(to_place_safe.points[1:])
        joined_trajectory.points.extend(to_place.points[1:])
        joined_trajectory.points.extend(back_to_place_safe.points[1:])
        joined_trajectory.points.extend(to_place_again.points[1:])
        joined_trajectory.points.extend(exit_from_place.points[1:])
        joined_trajectory.points.extend(back_to_safe.points[1:])

        results[str(index)] = {
            "pick_frame": pick_frame,
            "safe_frame": safe_frame,
            "place_frame": place_frame,
            "place_safe_frame": place_safe_frame,
            "pick_trajectory": pick_traj,
            "joined_trajectory": joined_trajectory
        }
        sticky["results"] = results
        sticky["last_status"] = f"Computed: {index}"
        print(f"✓ Computed complete trajectory for frame {index}")

    except Exception as e:
        print(f"Failed for frame {index}: {e}")
        print(traceback.format_exc())
        sticky["last_status"] = "Error: {}".format(str(e))
results = sticky.get("results", None)
results_list = [results]
if (
    results is not None
    and str(index) in results
    and "joined_trajectory" in results[str(index)]
):
    joined_trajectory = results[str(index)]["joined_trajectory"]
else:
    joined_trajectory = None


assembly = sticky.get("assembly", None)


ghenv.Component.Message = sticky["last_status"][:35]
print(sticky["last_status"])
sticks = sticky.get("geometry", None)

if reset:
    sticky["assembly"] = None
    sticky["geometry"] = None
    sticky["results"] = {}
