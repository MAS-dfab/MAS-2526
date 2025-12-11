from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import intersection_line_plane, Scale
from compas.geometry import angle_vectors
import math

def _calculate_z_vector_from_centerline(centerline_vector):
    z = Vector(0, 0, 1)
    angle = angle_vectors(z, centerline_vector)
    if angle < 0.1 or angle > math.pi - 0.001:
        z = Vector(1, 0, 0)
    return z

class Stick:
    # class attributes
    SIZE = 18.0
    # rectangular cross-section defaults (width x depth)
    WIDTH = SIZE
    DEPTH = SIZE

    # constructor with axis and rectangular dimensions
    def __init__(self, axis, width=None, depth=None, zvector=None):
        self.axis = axis
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self._get_stick_frame() if zvector is None else Frame(self.axis.midpoint, self.axis.direction, zvector)

    def _get_stick_frame(self):
        normal = _calculate_z_vector_from_centerline(self.axis.direction)
        frame = Frame(self.axis.midpoint, self.axis.direction, normal)
        return frame
    
    @property
    def geometry(self):
        """
        Computes the geometry of the stick as a Box based on its axis and size.
        Returns:
            Box: A compas box object representing the stick's geometry.
        """
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box


    def shift(self, length):
        """
        Shifts the stick along its axis by a specified length.
        Args:
            length (float): The amount to shift the stick along its axis.
        """
        self.axis.end = self.axis.end + self.axis.direction * length
        self.axis.start = self.axis.start + self.axis.direction * length

    def rotate_stick(self, angle, rotation_axis=None):
        """
        Rotates the stick around its axis by a specified angle.
        Args:
            angle (float): The angle in radians for the stick's rotation.
            rotation_axis (Vector, optional): The axis of rotation. If not provided,
                defaults to a vector perpendicular to the stick's axis in the z-direction.
        """
        if not rotation_axis:
            rotation_axis = self.axis.direction.cross(Vector(0,0,1))
        R = Rotation.from_axis_and_angle(rotation_axis, angle, self.axis.midpoint)
        self.axis.transform(R)

    def set_length(self, length):
        """
        Sets the length of the stick by adjusting the start position of the axis.
        Args:
        - length (float): The new length to set for the stick.

        """
        self.axis.start = self.axis.end - self.axis.direction * length
        if self.frame is not None:
            self.frame.point = self.axis.midpoint

    def scale(self, factor):
        """
        Scales the stick by a specified factor.

        Args:
            factor (float): The factor by which to scale the stick.
        """
        S = Scale.from_factors(
            [factor, factor, factor],
            frame=Frame.worldXY(),
        )
        self.axis.transform(S)
        self.width = self.width * factor
        self.depth = self.depth * factor


