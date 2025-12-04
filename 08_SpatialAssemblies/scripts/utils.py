"""
This module contains utility functions:
    1) Transformation functions
    2) Useful geometry functions e.g. Intersections
"""

import compas.geometry as cg
import math

# ----- Coordinate System conversions -----
def calculate_blend_radius(way_planes, radius):
    radii = []
    for i,frame in enumerate(way_planes[:-1]):
        dist = frame.point.distance_to_point(way_planes[i+1].point)
        if abs(radius)>dist/2:
            radius = dist/2
        else:
            radius = radius
        radii.append(radius)
    radii.append(0.0) #last point should be zero
    return radii
        

def rhino_to_robotbase(ref_plane,robot_base):
    """
    Function that transforms a reference plane from Rhino coordinate system to the robot's base coordinate system
    TODO (Jason): maybe change this whole method? maybe not model but robot base
    
    Args:
        ref_plane: compas.geometry Frame object. The reference plane to transform
        robot_base: compas.geometry Frame object. The base plane for building on. Given in robot's base coordinate system.
    
    Returns:
        ref_plane: Reference plane transformed to robot space
    """
    
    # Transform the orientation plane based on robot_base coordinate system
    _matrix = cg.Transformation.from_frame_to_frame(cg.Frame.worldXY(), robot_base)
    
    ref_plane.transform(_matrix)
    return ref_plane

def matrix_to_axis_angle(m):
    """
    Function that transforms a 4x4 matrix to axis-angle format
    referenced from Martin Baker's www.euclideanspace.com
    
    Args:
        m: compas.geometry Transformation object - 4x4 matrix
    
    Returns:
        axis: compas.geometry Vector object - axis-angle notation
    """
    
    epsilon = 0.01
    epsilon2 = 0.01
    
    # Access matrix elements using array indexing
    if (math.fabs(m[0,1] - m[1,0]) < epsilon) & (math.fabs(m[0,2] - m[2,0]) < epsilon) & (math.fabs(m[1,2] - m[2,1]) < epsilon):
    #singularity found
    #first check for identity matrix which must have +1 for all terms
    #in leading diagonal and zero in other terms
        if (math.fabs(m[0,1] + m[1,0]) < epsilon2) & (math.fabs(m[0,2] + m[2,0]) < epsilon2) & (math.fabs(m[1,2] + m[2,1]) < epsilon2) & (math.fabs(m[0,0] + m[1,1] + m[2,2] - 3) < epsilon2):
            #this singularity is identity matrix so angle = 0   make zero angle, arbitrary axis
            angle = 0
            x = 1
            y = z = 0
        else:
            # otherwise this singularity is angle = 180
            angle = math.pi;
            xx = (m[0,0] + 1) / 2
            yy = (m[1,1] + 1) / 2
            zz = (m[2,2] + 1) / 2
            xy = (m[0,1] + m[1,0]) / 4
            xz = (m[0,2] + m[2,0]) / 4
            yz = (m[1,2] + m[2,1]) / 4
            if ((xx > yy) & (xx > zz)):
                # m[0,0] is the largest diagonal term
                if (xx < epsilon):
                    x = 0
                    y = z = 0.7071
                else:
                    x = math.sqrt(xx)
                    y = xy / x
                    z = xz / x
            elif (yy > zz): 
                # m[1,1] is the largest diagonal term
                if (yy < epsilon):
                    x = z = 0.7071
                    y = 0
                else: 
                    y = math.sqrt(yy)
                    x = xy / y
                    z = yz / y
            else: 
                # m[2,2] is the largest diagonal term so base result on this
                if (zz < epsilon):
                    x = y = 0.7071
                    z = 0
                else:
                    z = math.sqrt(zz)
                    x = xz / z
                    y = yz / z
    else:
        s = math.sqrt((m[2,1] - m[1,2]) * (m[2,1] - m[1,2])+ (m[0,2] - m[2,0]) * (m[0,2] - m[2,0])+ (m[1,0] - m[0,1]) * (m[1,0] - m[0,1])); # used to normalise
        if (math.fabs(s) < 0.001):
            #prevent divide by zero, should not happen if matrix is orthogonal and should be
            s = 1
        angle = math.acos((m[0,0] + m[1,1] + m[2,2] - 1) / 2)
        x = (m[2,1] - m[1,2]) / s
        y = (m[0,2] - m[2,0]) / s
        z = (m[1,0] - m[0,1]) / s
    angleRad = angle
    axis = cg.Vector(x, y, z)
    axis = axis * angleRad
    
    return axis

def matrix_to_euler(m):
    """
    Gets the Euler rotation angles from a transformation matrix
    from http://forums.codeguru.com/archive/index.php/t-329530.html
    
    Args:
        m = Transformation object
    Returns:
        tuple of euler angles in radians
    """
    
    rotz = math.atan2(m[1,0], m[0,0])
    roty = -math.asin(m[2,0])
    rotx = math.atan2(m[2,1], m[2,2])
    return (rotx, roty, rotz)

# ----- Matrix related helper functions

def dh_matrix(d, theta, a, alpha):
    """
    This function creates the Denavit Hartenberg transformation matrix between adjacent frames
    
    Arguments:
        d: Joint distance. in mm
        theta: joint angle. in radians
        a: link length. in mm
        alpha: twist angle. in radians
    
    Returns:
        m: Denavit Hartenberg transformation matrix
    """
    
    _matrix = [
    [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
    [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
    [0, math.sin(alpha), math.cos(alpha), d],
    [0, 0, 0, 1]
    ]
    
    m = cg.Transformation(_matrix)
            
    return m

def concatenate_matrices(matrices):
    """
    This function creates a concatenated matrix from a list of matrices
    
    Arguments:
        matrices: A list of tranformation matrices
    
    Returns:
        _transform: Concatenated matrix
    """
    _transform = matrices[0]
    for i in range(1,len(matrices)):
        _transform = _transform * matrices[i]
    return _transform


# ----- Miscellaneous geometry helper functions
def signed_angle(v1, v2, v_normal):
    """
    This function gets the angle between 2 vectors -pi < theta< pi
    
    Arguments:
        v1: Vector. First unitized vector
        v2: Vector. Second unitized vector
        v_normal: Vector. Normal to 2 vectors that determines what is positive/negative
    
    Returns:
        theta: float. signed angle between -pi and pi
    """
    # from 0 to pi
    c = v1.dot(v2)
    n = v1.cross(v2)
    s = n.length
    
    theta = math.atan2(s, c)
    
    if (n.dot(v_normal) < 0):
        theta *= -1
    return theta

def cir_cir_intersection(cir1, cir2):
    """
    Function that returns the intersection points between two circles
    
    Arguments:
        1) cir1: First circle (compas.geometry Circle)
        2) cir2: Second Circle (compas.geometry Circle)
    
    Returns:
        xpts: list of 2 Point objects
        
    Note that there is no error checking
    """
    r1 = cir1.radius
    r2 = cir2.radius
    d = cir1.frame.point.distance_to_point(cir2.frame.point)
    
    a = (r1 **2 - r2**2 + d**2)/(2*d)
    h = math.sqrt(r1 **2 - a **2 )
    
    v_c1 = cg.Vector.from_start_end(cg.Point(0,0,0), cir1.frame.point)
    v_c2 = cg.Vector.from_start_end(cg.Point(0,0,0), cir2.frame.point)
    
    v_c1c2 = v_c2 - v_c1
    v_c1c2.unitize()
    v_c1c2 = v_c1c2 * a
    
    v_pt0 = v_c1 + v_c1c2
    
    v_pt0ptX = cir1.frame.normal.cross(v_c1c2)
    v_pt0ptX.unitize()
    v_pt0ptX = v_pt0ptX * h
    
    xpt1 = cg.Point(v_pt0 + v_pt0ptX)
    v_pt0ptX = v_pt0ptX * -1
    xpt2 = cg.Point(v_pt0 + v_pt0ptX)

    return [xpt1, xpt2]
    
def check_arguments(function):
    def decorated(*args):
        if None in args:
            raise TypeError("Invalid Argument")
        return function(*args)
    return decorated