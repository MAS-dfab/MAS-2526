from compas.geometry import Point, Box, Frame, Vector, Plane, Rotation, Line
from compas.geometry import angle_vectors, intersection_line_plane
import math

def _calculate_z_vector_from_centerline(centerline_vector):
    c = Vector(0,0,1)
    angle = angle_vectors(c, centerline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        c = Vector(1,0,0)
    return c


class Stick:
    SIZE = 13.0

    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, axis, z_vector = None, width = None, depth = None):

        self.axis = axis
        self.z_vector = z_vector
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self._get_stick_frame()
        self.length = self.axis.length
    
    def _get_stick_frame(self):
        if self.z_vector:
            normal = self.z_vector
        else:
            normal = _calculate_z_vector_from_centerline(self.axis.direction)
        frame = Frame(self.axis.midpoint, self.axis.direction, normal)
        return frame

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box
    
    def rotate_stick(self, angle, rotation_axis=None, pt=None):
        if not rotation_axis:
            rotation_axis = self.axis.direction
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), pt or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)
    
    def eval_frame(self, face_index=0):
        """
        Gets a frame on one of the four faces of a stick.

        Args:
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face index.
        """
        # Rotate stick frame based on index
        base = self.frame.copy()
        base.point += self.axis.direction * 0.5  # Move to center of stick
        angle = (face_index % 4) * (math.pi / 2)
        R = Rotation.from_axis_and_angle(base.xaxis, angle, base.point)
        new_stick_frame = base.transformed(R)

        # Offset frame to be on surface of stick
        new_stick_frame.point += new_stick_frame.zaxis * (self.depth / 2)
        return new_stick_frame

def stick_bridge(stick0, stick1):
    plane0 = Plane.from_frame(stick0.frame)
    plane1 = Plane.from_frame(stick1.frame)
    p0 = intersection_line_plane(stick0.axis, plane1)
    p1 = intersection_line_plane(stick1.axis, plane0)

    return Stick(Line(p0, p1))