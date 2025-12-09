from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box, Transformation

from sticks_251207 import Stick
import math


class StickModuleFJ:
    def __init__(self, frame, stick_width, stick_depth, stick_length):
        self.frame = frame
        self.width = stick_width
        self.depth = stick_depth
        self.length = stick_length

        self.sticks = []
    
    def create_module_c(self):
        # move stick in x
        offsetpt_xa = (self.frame.point - self.frame.xaxis*self.width*3.5 - self.frame.yaxis*self.width)
        offsetpt_xb = (self.frame.point - self.frame.xaxis*self.width*3.5 + self.frame.yaxis*self.width)
        
        offsetpt_ya = (self.frame.point - self.frame.yaxis*self.width*4.5-self.frame.zaxis*self.depth)
        offsetpt_yb = (self.frame.point - self.frame.yaxis*self.width*4.5+self.frame.zaxis*self.depth)
     
        offsetpt_z = (self.frame.point + self.frame.xaxis*self.length-self.frame.xaxis*self.width*7 - self.frame.zaxis*self.width*3.5)
        
        stick_xa = Stick(Line(offsetpt_xa, offsetpt_xa+self.frame.xaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_xa)
        
        stick_xb = Stick(Line(offsetpt_xb, offsetpt_xb+self.frame.xaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_xb)
        
        stick_ya = Stick(Line(offsetpt_ya, offsetpt_ya + self.frame.yaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_ya)

        stick_yb = Stick(Line(offsetpt_yb, offsetpt_yb+self.frame.yaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_yb)

        stick_z = Stick(Line(offsetpt_z, offsetpt_z+self.frame.zaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_z)

class StickModuleA:
    
    LENGTH = 200
    
    def __init__(self, plane, angle):
        self.plane = plane
        self.angle = angle
        self.length = StickModuleA.LENGTH
        self.sticks = {"stick1": None, "stick2": None, "stick3": None}
        # self.sticks = []
        
    def create_module(self):
        """
        Docstring for create_module
        
        :param self: angle of the stick one and stick two and legth
        :param plane: plane where the module is created
        """
        base_frame = self.plane
        
        # Stick one
        line1 = Line.from_point_and_vector(base_frame.point, base_frame.zaxis * self.length)
        stick1 = Stick(line1, base_frame)
        self.sticks["stick1"] = stick1
        
        # Stick two
        offset_stick_two = base_frame.yaxis * Stick.WIDTH
        base_point = line1.midpoint - (base_frame.xaxis * (self.length / 2)) - offset_stick_two
        line2 = Line.from_point_and_vector(base_point, base_frame.xaxis * self.length)
        stick2 = Stick(line2, base_frame)
        self.sticks["stick2"] = stick2
        
        # Stick three
        offset_stick_three = base_frame.zaxis * Stick.WIDTH
        line_3 = line2.rotated(math.radians(self.angle), base_frame.zaxis, line1.midpoint)    
        line_3.translate(offset_stick_three)
        line_3.translate(-base_frame.yaxis * (Stick.WIDTH / 2))
        stick3 = Stick(line_3, base_frame)
        self.sticks["stick3"] = stick3
        
        return self.sticks