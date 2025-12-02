from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box
from group_one_sticks import Stick
import math

def rotate_move(point, plane, angle, rotation_axis, move_axis, length):
    my_frame = Frame(point, plane.xaxis, plane.yaxis)
    if rotation_axis == "x":
        rotated_plane = my_frame.rotated(angle, my_frame.xaxis, my_frame.point)
    elif rotation_axis == "y":
        rotated_plane = my_frame.rotated(angle, my_frame.yaxis, my_frame.point)
    else:
        rotated_plane = my_frame
    
    if move_axis == "x":
        v_a = rotated_plane.xaxis
    elif move_axis == "y":
        v_a = rotated_plane.yaxis
    else:
        v_a = rotated_plane.zaxis
    
    v_a.unitize()
    v_a = v_a.scaled(length)
    moved_point = point.translated(v_a)
    
    return moved_point


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
        plane_a.rotate(math.radians(angle_a))
        
        point_b = point_a.translated((self.frame.yaxis * Stick.WIDTH) + (self.frame.xaxis * sticks_distance))
        plane_b = Frame(point_a, self.frame.xaxis, self.frame.yaxis)
        plane_b.rotate(math.radians(-angle_b))

        # create lines
        line_a = Line.from_point_and_vector(point_a, plane_a.zaxis * length)
        line_b = Line.from_point_and_vector(point_b, plane_b.zaxis * length)
        
        # use stick class
        s1 = Stick(line_a)
        s2 = Stick(line_b)
        stick1 = s1.geometry
        stick2 = s2.geometry
        
        return [stick1, stick2]
    
    
        # def CreateModule(self, angle_a = 45, angle_b = 45, length = 2, sticks_distance = 5):
    #     """
    #     Docstring for CreateModule
        
    #     :param self: angle of the stick one and stick two and legth
    #     :return: two lines
    #     """
    #     # line initial planes that are rotated
    #     point_a1 = self.point
    #     point_a2 = rotate_move(self.point, self.frame, math.radians(angle_a), "y", "z",length)
        
    #     point_b = rotate_move(self.point, self.frame, math.radians(0), 0, "x",sticks_distance)
    #     point_b_translate = rotate_move(point_b, self.frame, math.radians(0), 0, "y",Stick.WIDTH)
    #     point_b2 = rotate_move(point_b_translate, self.frame, math.radians(-angle_b), "y", "z",length)

    #     # create lines
    #     line_a = Line(point_a1, point_a2)
    #     line_b = Line(point_b_translate, point_b2)
        
    #     # use stick class
    #     s1 = Stick(line_a)
    #     s2 = Stick(line_b)
    #     stick1 = s1.geometry
    #     stick2 = s2.geometry
        
    #     return [stick1, stick2]