from compas.geometry import Line, Frame, Vector

from group_one_sticks import Stick
import math

from compas.geometry import Rotation


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

