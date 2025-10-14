"""
This module wraps standard UR Script functions.
Main change is that plane information substitutes for pose data.
"""

import utils
import compas.geometry as cg

# Some Constants
MAX_ACCEL = 3
MAX_VELOCITY = 4

def move_l(plane_to, accel, vel):
    """
    Function that returns UR script for linear movement in tool-space.

    Args:
        plane_to: compas.geometry.Frame. A target frame for calculating pose (in UR base coordinate system)
        accel: tool accel in m/s^2
        vel: tool speed in m/s

    Returns:
        script: UR script
    """

    # Check acceleration and velocity are non-negative and below a set limit
    accel = MAX_ACCEL if (abs(accel) > MAX_ACCEL) else abs(accel)
    vel = MAX_VELOCITY if (abs(vel) > MAX_VELOCITY) else abs(vel)

    _matrix = cg.Transformation.from_frame_to_frame(cg.Frame.worldXY(), plane_to)
    _axis_angle = utils.matrix_to_axis_angle(_matrix)
    # Create pose data
    _pose = [
        plane_to.point.x / 1000,
        plane_to.point.y / 1000,
        plane_to.point.z / 1000,
        _axis_angle[0],
        _axis_angle[1],
        _axis_angle[2],
    ]
    _pose_fmt = "p[" + ("%.4f," * 6)[:-1] + "]"
    _pose_fmt = _pose_fmt % tuple(_pose)
    # Format UR script
    script = "movel(%s, a = %.2f, v = %.2f)\n" % (_pose_fmt, accel, vel)
    return script


def move_l_blend(plane_to, accel, vel, blend_radius=0):
    """
    Function that returns UR script for linear movement in tool-space.

    Args:
        plane_to: compas.geometry.Frame. A target frame for calculating pose (in UR base coordinate system)
        accel: tool accel in m/s^2
        vel: tool speed in m/s

    Returns:
        script: UR script
    """

    # Check acceleration and velocity are non-negative and below a set limit
    accel = MAX_ACCEL if (abs(accel) > MAX_ACCEL) else abs(accel)
    vel = MAX_VELOCITY if (abs(vel) > MAX_VELOCITY) else abs(vel)
    # Check blend radius is positive
    blend_radius = max(0, blend_radius)
    blend_radius = blend_radius / 1000

    _matrix = cg.Transformation.from_frame_to_frame(cg.Frame.worldXY(), plane_to)
    _axis_angle = utils.matrix_to_axis_angle(_matrix)
    # Create pose data
    _pose = [
        plane_to.point.x / 1000,
        plane_to.point.y / 1000,
        plane_to.point.z / 1000,
        _axis_angle[0],
        _axis_angle[1],
        _axis_angle[2],
    ]
    _pose_fmt = "p[" + ("%.4f," * 6)[:-1] + "]"
    _pose_fmt = _pose_fmt % tuple(_pose)
    # Format UR script
    script = "movel(%s, a = %.2f, v = %.2f, r = %.4f)\n" % (_pose_fmt, accel, vel, blend_radius)
    return script


def move_j(joints, accel, vel):
    """
    Function that returns UR script for linear movement in joint space.

    Args:
        joints: A list of 6 joint angles (double).
        accel: tool accel in m/s^2
        vel: tool speed in m/s

    Returns:
        script: UR script
    """
    # Check acceleration and velocity are non-negative and below a set limit

    _j_fmt = "[" + ("%.2f," * 6)[:-1] + "]"
    _j_fmt = _j_fmt % tuple(joints)
    script = "movej(%s, a = %.2f, v = %.2f)\n" % (_j_fmt, accel, vel)
    return script


def set_tcp_by_plane(x_offset, y_offset, z_offset, ref_plane=cg.Frame.worldXY()):
    """
    Function that returns UR script for setting tool center point.

    Args:
        x_offset: float. tooltip offset in mm
        y_offset: float. tooltip offset in mm
        z_offset: float. tooltip offset in mm
        ref_plane: Frame that defines orientation of the tip. If none specified, world XY frame used as default. (in UR base coordinate system)

    Returns:
        script: UR script
    """

    if ref_plane != cg.Frame.worldXY():
        _matrix = cg.Transformation.from_frame_to_frame(cg.Frame.worldXY(), ref_plane)
        _axis_angle = utils.matrix_to_axis_angle(_matrix)
    else:
        _axis_angle = [0, 0, 0]
    # Create pose data
    _pose = [
        x_offset / 1000,
        y_offset / 1000,
        z_offset / 1000,
        _axis_angle[0],
        _axis_angle[1],
        _axis_angle[2],
    ]
    _pose_fmt = "p[" + ("%.4f," * 6)[:-1] + "]"
    _pose_fmt = _pose_fmt % tuple(_pose)

    # Format UR script
    script = "set_tcp(%s)\n" % (_pose_fmt)
    return script


def set_tcp_by_angles(x_offset, y_offset, z_offset, x_rotate, y_rotate, z_rotate):
    """
    Function that returns UR script for setting tool center point using rotation angles.

    Args:
        x_offset: float. tooltip offset in mm
        y_offset: float. tooltip offset in mm
        z_offset: float. tooltip offset in mm
        x_rotate: float. rotation around world x axis in radians
        y_rotate: float. rotation around world y axis in radians
        z_rotate: float. rotation around world z axis in radians

    Returns:
        script: UR script
    """

    # Create rotation matrix
    _rX = cg.Rotation.from_axis_and_angle([1, 0, 0], x_rotate)
    _rY = cg.Rotation.from_axis_and_angle([0, 1, 0], y_rotate)
    _rZ = cg.Rotation.from_axis_and_angle([0, 0, 1], z_rotate)
    _r = _rX * _rY * _rZ
    _axis_angle = utils.matrix_to_axis_angle(_r)

    # Create pose data
    _pose = [
        x_offset / 1000,
        y_offset / 1000,
        z_offset / 1000,
        _axis_angle[0],
        _axis_angle[1],
        _axis_angle[2],
    ]
    _pose_fmt = "p[" + ("%.4f," * 6)[:-1] + "]"
    _pose_fmt = _pose_fmt % tuple(_pose)

    # Format UR script
    script = "set_tcp(%s)\n" % (_pose_fmt)
    return script


def popup(message, title):
    """
    Function that returns UR script for popup.

    Args:
        message: str. Message to display.
        title: str. Title of the popup.

    Returns:
        script: UR script
    """
    script = 'popup("%s","%s") \n' % (message, title)
    return script


def sleep(time):
    """
    Function that returns UR script for sleep().

    Args:
        time: float. Time in seconds.

    Returns:
        script: UR script
    """
    script = "sleep(%s) \n" % (time)
    return script


def set_digital_out(id, signal):
    """
    Function that returns UR script for setting digital out.

    Args:
        id: int. Input id number.
        signal: bool. Signal level - on or off.

    Returns:
        script: UR script
    """

    # Format UR script
    script = "set_digital_out(%s,%s)\n" % (id, signal)
    return script