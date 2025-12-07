from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box
from group_one_sticks import Stick
import math

class StickModule:
    
    LENGTH = 200
    
    def __init__(self, angle_a = 45, angle_b = 45, sticks_distance = 5):
        self.angle_a = angle_a
        self.angle_b = angle_b
        self.length = StickModule.LENGTH
        self.stick_distance = sticks_distance
        self.sticks = []

        
    def CreateModule(self, plane):
        """
        Docstring for CreateModule
        
        :param self: angle of the stick one and stick two and legth
        :return: two lines
        """
        # line initial planes that are rotated
        point_a = plane.point
        plane_a = Frame(point_a, plane.xaxis, plane.yaxis)
        new_plane_a = plane_a.rotated(math.radians(self.angle_a), plane_a.yaxis, point_a)
        
        point_b = point_a.translated((plane.yaxis * Stick.WIDTH) + (plane.xaxis * self.stick_distance))
        plane_b = Frame(point_a, plane.xaxis, plane.yaxis)
        new_plane = plane_b.rotated(math.radians(-self.angle_b), plane_b.yaxis, point_b)

        # create lines
        line_a = Line.from_point_and_vector(point_a, new_plane_a.zaxis * self.length)
        line_b = Line.from_point_and_vector(point_b, new_plane.zaxis * self.length)
        
        # use stick class
        s1 = Stick(line_a)
        s2 = Stick(line_b)
        stick1 = s1.geometry
        stick2 = s2.geometry
        
        return [s1, s2]
