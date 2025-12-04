from compas.geometry import Plane, Box, Line, Vector, Frame
from compas.geometry import angle_vectors, Rotation
import math


class Stick:
    size = 13.0
    width = size
    depth = size
    def __init__(self, frame, length=None, width=None, depth=None):
        self.frame = frame
        self.length = length or 200.0
        self.width = width or Stick.width
        self.depth = depth or Stick.depth

        self.axis = self._axis_from_frame()
        self.midframe = self._get_axis_mid_frame()
        self.evalframe = self.eval_frame()

        self.sticks = []

    def _axis_from_frame(self):
        start_pt = self.frame.point
        direction = self.frame.xaxis
        axis = Line.from_point_direction_length(start_pt, direction, self.length)
        return axis
    
    def _get_axis_mid_frame(self):
        vector = self.frame.xaxis.unitized()
        vector *= (self.length / 2)
        frame = self.frame.translated(vector)
        return frame
    
    def eval_frame(self, face_index=0, t_value=.5, z_offset=False):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            face_index: Face index (0-3) around the stick
            t_value: The relative position along the line as a fraction of the length of the line. 0.0 corresponds to the start point and 1.0 corresponds to the end point. Numbers outside of this range are also valid and correspond to points beyond the start and end point.
            
        Returns:
            Frame on the specified face and t value
        """

        # Rotate stick frame based on index
        base = self.midframe
        angle = (face_index % 4) * (math.pi / 2)
        R = Rotation.from_axis_and_angle(base.xaxis, angle, base.point)
        new_stick_frame = base.transformed(R)
        new_stick_frame.point = self.axis.point_at(t_value)
        # Offset frame to be on surface on stick
        if z_offset is False:
            offset = self.depth / 2
        else:
            offset = -self.depth / 2
        new_stick_frame.point += new_stick_frame.zaxis * (offset)
        return new_stick_frame
    
    @property
    def geometry(self):
        # return Box(self.length, self.width, self.depth, self.midframe)
        geo = Box(self.length, self.width, self.depth, self.midframe)
        return geo
    
    def visualize(self):
        # return [stick.geometry for stick in self.sticks]
        return self.sticks
