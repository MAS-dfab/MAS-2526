from compas.geometry import Line
from compas.geometry import Frame
from compas.geometry import Vector
from compas.geometry import Plane
from compas.geometry import Rotation
from compas.geometry import intersection_plane_plane
import math

from Sticks import Stick

class GrowTowards:
    def __init__(self, root_frame, target_frame, stick_length=None, width=None, depth=None):
        """
        Constructor for Bridge module
        
        Args:
            root_frame: starting frame derived from RootModule
            target_frame: destination frame input

            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """

        self.root_frame = root_frame
        self.target_frame = target_frame

        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.normal_deviation = self.compare_angles(self.root_frame, self.target_frame)

        self.frame_intersection = self.get_frame_intersection(self.root_frame, self.target_frame)

    def get_frame_intersection(self, frame_0, frame_1):
        plane_0 = Plane.from_frame(frame_0)
        plane_1 = Plane.from_frame(frame_1)
        int_pts = intersection_plane_plane(plane_0, plane_1)
        int_line = Line(int_pts[0],int_pts[1])
        return int_line
    
    def compare_angles(frame_0, frame_1):
        normal_deviation = Vector.angle_vectors(frame_0.normal, frame_1.normal)
        return normal_deviation