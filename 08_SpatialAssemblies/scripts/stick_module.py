from compas.geometry import Line, Frame, Vector

from Sticks import Stick


class OStickModule:
    def __init__(self, pt, stick_length, stick_width, stick_depth):
        self.pt = pt
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth

        self.sticks = []

    def create_orthogonal_module(self, type = {"x":0, "y":0, "z":0}):

        #stick x
        offsetpt_x = (self.pt 
                      - Vector(self.depth/2,0,0)
                      + Vector(0,2* self.depth * type["x"],0))
        
        stick_x = Stick(Line(offsetpt_x,offsetpt_x + Vector(self.length,0,0)), width=self.width, depth=self.depth)
        if type["x"] != 2:
            self.sticks.append(stick_x)

        #stick y
        offsetpt_y = (self.pt 
                      - Vector(0, self.depth/2,0)
                      + Vector(0,0,self.depth)
                      + Vector(2* self.depth * type["y"],0,0))
        
        stick_y = Stick(Line(offsetpt_y, offsetpt_y + Vector(0,self.length,0)), Vector(1,0,0), self.width, self.depth)
        if type["y"] != 2:
            self.sticks.append(stick_y)

        #stick z
        offsetpt_z = (self.pt 
                      + Vector(0,self.depth,0)
                      + Vector(self.depth,0,0)
                      - Vector(0,0,self.depth/2)
                      - Vector(0, 2*self.depth * type["z"],0))
        
        stick_z = Stick(Line(offsetpt_z,offsetpt_z + Vector(0,0,self.length)), width = self.width,depth =self.depth)
        if type["z"] != 2:
            self.sticks.append(stick_z)