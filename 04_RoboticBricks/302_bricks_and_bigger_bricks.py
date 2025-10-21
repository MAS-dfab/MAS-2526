import compas_rrc as rrc
from compas.data import json_load
import math

from compas.geometry import Frame, Point, Vector




############## Constants

PICK_FRAME = Frame(
    point=Point(x=-1258.596, y=-890.31, z=-107.768),
    xaxis=Vector(x=-1.000, y=-0.016, z=0.003),
    yaxis=Vector(x=-0.016, y=1.000, z=-0.011),
)

HOME_CONFIG = [60.0, -15, 60, 0, 45, -120]
SAFE_EXT_AXES = [-1590.0]

io_open = "doUnitR32ValveB1"
io_close = "doUnitR32ValveA1"

############## Functions
def get_safe_frame(frame, offset=300.0):
    pick_frame = Frame(frame.point, frame.xaxis, frame.yaxis)
    safety_frame = pick_frame.translated(Vector(0, 0, offset))
    return safety_frame


def pick_a_brick():
    # Set work object
    abb.send_and_wait(rrc.SetWorkObject("wobj0"))
    # Ensure gripper is open
    abb.send_and_wait(rrc.PrintText("Opening gripper to ensure it's open"))
    open_gripper(abb)
    safe_pick_frame = get_safe_frame(PICK_FRAME, offset=300.0)
    # Go to home position
    abb.send_and_wait(rrc.PrintText("Moving to home configuration"))
    abb.send_and_wait(
        rrc.MoveToJoints(
            HOME_CONFIG, ext_axes=SAFE_EXT_AXES, speed=500, zone=rrc.Zone.FINE
        )
    )
    # Move to safe pick frame
    # Write message
    abb.send_and_wait(rrc.PrintText("Moving to safe pick frame"))
    print(safe_pick_frame)
    abb.send_and_wait(
        rrc.MoveToRobtarget(
            safe_pick_frame, ext_axes=SAFE_EXT_AXES, speed=500, zone=rrc.Zone.FINE
        )
    )
    # Move to pick frame
    abb.send_and_wait(rrc.PrintText("Moving to pick frame"))
    abb.send_and_wait(
        rrc.MoveToRobtarget(
            PICK_FRAME, ext_axes=SAFE_EXT_AXES, speed=50, zone=rrc.Zone.FINE
        )
    )
    # Close gripper
    abb.send_and_wait(rrc.PrintText("Closing gripper to pick brick"))
    close_gripper(abb)
    # Move back to safe pick frame
    abb.send_and_wait(rrc.PrintText("Moving back to safe pick frame"))
    abb.send_and_wait(
        rrc.MoveToRobtarget(
            safe_pick_frame, ext_axes=SAFE_EXT_AXES, speed=500, zone=rrc.Zone.FINE
        )
    )
    # Move to home position
    abb.send_and_wait(rrc.PrintText("Returning to home position"))
    abb.send_and_wait(
        rrc.MoveToJoints(
            HOME_CONFIG, ext_axes=SAFE_EXT_AXES, speed=500, zone=rrc.Zone.FINE
        )
    )


def place_a_brick(frame, ext_axes, wobj="w_pallet0"):
    # Set work object
    abb.send_and_wait(rrc.SetWorkObject(wobj))
    print("Placing brick at frame:", frame)
    # Move to safe place frame
    safe_place_frame = get_safe_frame(frame, offset=300.0)
    # Move to safe place frame
    abb.send_and_wait(rrc.PrintText("Moving to safe place frame"))
    abb.send_and_wait(
        rrc.MoveToRobtarget(
            safe_place_frame, ext_axes=ext_axes, speed=500, zone=rrc.Zone.FINE
        )
    )
    # Move to place frame
    abb.send_and_wait(rrc.PrintText("Moving to place frame"))
    abb.send_and_wait(
        rrc.MoveToRobtarget(frame, ext_axes=ext_axes, speed=50, zone=rrc.Zone.FINE)
    )
    # Open gripper
    abb.send_and_wait(rrc.PrintText("Opening gripper to release brick"))
    open_gripper(abb)
    # Move back to safe place frame
    abb.send_and_wait(rrc.PrintText("Moving back to safe place frame"))
    abb.send_and_wait(
        rrc.MoveToRobtarget(
            safe_place_frame, ext_axes=ext_axes, speed=500, zone=rrc.Zone.FINE
        )
    )
    # Move to home position
    abb.send_and_wait(rrc.PrintText("Returning to home position"))
    abb.send_and_wait(
        rrc.MoveToJoints(
            HOME_CONFIG, ext_axes=ext_axes, speed=500, zone=rrc.Zone.FINE
        )
    )


def open_gripper(abb):
    abb.send_and_wait(rrc.SetDigital(io_close, 0))  # Close gripper to ensure it's closed
    abb.send_and_wait(rrc.SetDigital(io_open, 1))  # Open gripper to ensure it's open
    abb.send_and_wait(rrc.WaitTime(0.5))  # Wait for gripper


def close_gripper(abb):
    abb.send_and_wait(rrc.SetDigital(io_open, 0))  # Close gripper to ensure it's closed
    abb.send_and_wait(rrc.SetDigital(io_close, 1))  # Close gripper to pick brick
    abb.send_and_wait(rrc.WaitTime(0.5))  # Wait for gripper


def fine_adjust_z(frame, adjustment):
    adjusted_frame = Frame(frame.point, frame.xaxis, frame.yaxis)
    adjusted_frame.point.z += adjustment
    return adjusted_frame


def get_external_axes_for_wobj(wobj):
    if wobj == "w_pallet0":
        ext_axes = [-1590.0]
    elif wobj == "w_pallet1":
        ext_axes = [-1000.0]
    elif wobj == "w_pallet2":
        ext_axes = [-600.0]
    else:
        ext_axes = SAFE_EXT_AXES

    return ext_axes


def load_brick_frames(filename):
    data = json_load(filename)
    return data

############ Main Script
# Import JSON file
from pathlib import Path

current_directory = Path(__file__).resolve().parent / "data"
data_file = current_directory / "out_brickframes.json"

data = json_load(data_file)

try:
    ros = rrc.RosClient()
    ros.run()

    # robot 12 to move tool and external axes 1
    abb = rrc.AbbClient(ros, "/rob1")
    abb.send_and_wait(rrc.SetTool("t_BrickGripper"))
    abb.send_and_wait(rrc.SetAcceleration(100, 50))
    abb.send_and_wait(rrc.PrintText(("Starting ABB Robot")))

    abb.send_and_wait(rrc.SetWorkObject("wobj0"))

    wobj = data["wobj"]
    brick_frames = data["frames"]
    ext_axes = get_external_axes_for_wobj(wobj)

    for frame in brick_frames:
        frame = fine_adjust_z(frame, adjustment=3.0)
        pick_a_brick()
        place_a_brick(frame, ext_axes, wobj=wobj)


except Exception as e:
    print(e)

finally:
    ros.close()
