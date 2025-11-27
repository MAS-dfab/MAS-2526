from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import angle_vectors, Rotation
import math

def _calculate_z_vector_from_centerline(centerline_vector):
    c = Vector(0,1,0)
    angle = angle_vectors(c, centerline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        c = Vector(1,0,0)
    return c

class Stick:
    size = 13.0
    width = size
    depth = size
    def __init__(self, axis, width=None, depth=None, z_vector=None):
        self.z_vector = z_vector
        self.axis = axis
        self.width = width or Stick.width
        self.depth = depth or Stick.depth
        self.midframe = self.get_mid_frame()
        self.evalframe = self.eval_frame()

    def get_mid_frame(self):
        if self.z_vector:
            normal = self.z_vector
        else:
            normal = _calculate_z_vector_from_centerline(self.axis.direction)
        frame = Frame(self.axis.midpoint, self.axis.direction, normal)
        return frame
    
    def eval_frame(self, face_index=0, t=.5):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            stick_index: Index of the stick
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face
        """

        # Rotate stick frame based on index
        base = self.midframe
        face_index = face_index % 4
        angle = face_index * (math.pi / 2)
        R = Rotation.from_axis_and_angle(base.xaxis, angle, base.point)
        new_stick_frame = base.transformed(R)
        eval_pt = self.axis.point_at(t)
        new_stick_frame.point = eval_pt
        # Offset frame to be on surface on stick
        new_stick_frame.point += new_stick_frame.zaxis * (self.depth / 2)
        return new_stick_frame
    
    @property
    def geometry(self):
        return Box(self.axis.length, self.width, self.depth, self.midframe)