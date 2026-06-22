from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import intersection_line_plane, Scale
from compas.geometry import angle_vectors, Rotation
import math

def _calculate_z_vector_from_conterline(conterline_vector):
    vec = Vector(0, 0, 1)
    angle = angle_vectors(vec, conterline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        vec = Vector(1, 0, 0)
    return vec
    
class Stick:
    size = 13.0
    width = size
    depth = size
    def __init__(self, axis, z_vector = None, width = None, depth = None):
        self.axis = axis
        self.z_vector = z_vector
        self.width = width or Stick.width
        self.depth = depth or Stick.depth
        self.frame = self.get_stick_frame()

    def get_stick_frame(self): 
        if self.z_vector:
            normal = self.z_vector
        else:
            normal = _calculate_z_vector_from_conterline(self.axis.direction)
        frame = Frame(self.axis.midpoint, self.axis.direction, normal)
        return frame
    
    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)

        return box

    def rotate_stick(self, angle, rotation_axis=None, point=None):
        if not rotation_axis:
            rotation_axis = self.axis.direction
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), point or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)



def stick_bridge(stick0, stick1):
    plane0 = Plane(stick0.axis.midpoint, stick0.frame.normal)
    p1 = intersection_line_plane(stick1.axis, plane0)
    
    plane1 = Plane(stick1.axis.midpoint, stick1.frame.normal)
    p0 = intersection_line_plane(stick0.axis, plane1)
    
    return Stick(Line(p0, p1))
    
    
