from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box
from group_one_sticks import Stick
import math

class StickModule:
    
    def __init__(self, plane):
        self.frame = plane
        self.point = plane.point
        self.sticks = []

        
    def CreateModule(self, angle_a = 45, angle_b = 45, length = 2, sticks_distance = 5):
        """
        Docstring for CreateModule
        
        :param self: angle of the stick one and stick two and legth
        :return: two lines
        """
        # line initial planes that are rotated
        point_a = self.point
        plane_a = Frame(point_a, self.frame.xaxis, self.frame.yaxis)
        new_plane_a = plane_a.rotated(math.radians(angle_a), plane_a.yaxis, point_a)
        
        point_b = point_a.translated((self.frame.yaxis * Stick.WIDTH) + (self.frame.xaxis * sticks_distance))
        plane_b = Frame(point_a, self.frame.xaxis, self.frame.yaxis)
        new_plane = plane_b.rotated(math.radians(-angle_b), plane_b.yaxis, point_b)

        # create lines
        line_a = Line.from_point_and_vector(point_a, new_plane_a.zaxis * length)
        line_b = Line.from_point_and_vector(point_b, new_plane.zaxis * length)
        
        # use stick class
        s1 = Stick(line_a)
        s2 = Stick(line_b)
        stick1 = s1.geometry
        stick2 = s2.geometry
        
        return [stick1, stick2]
