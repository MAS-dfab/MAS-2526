from compas.geometry import Line, Frame, Vector
from compas.geometry import Rotation
from sticks_251207 import Stick
import math


class BranchStickModulesUpdates:
    def __init__(self, root_frame, stick_length = None, width=None, depth=None, rot_angle=0):
        self.root_frame = root_frame
        self.modules = []
        self.stick_length = stick_length
        self.width = width or  Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.rot_angle = rot_angle
        self._init_first_module(root_frame)
    
    def _init_first_module(self, frame):
        my_module = StickModule(frame, self.width, self.depth, self.stick_length, rot_angle=self.rot_angle)

        # my_module.create_module_a() 
        my_module.create_module_a()
        self.modules.append(my_module)
    
    def get_face_frame(self, module_index, stick_index, face_index):
        module = self.modules[module_index]

        stick_frame = module.sticks[stick_index].frame

        angle = face_index*math.pi/2 # research, 0, 1, 2,3 is 90 degree steps
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)

        new_frame.point = module.sticks[stick_index].axis.end

        new_frame.point += new_frame.yaxis * self.depth/2
        
        return new_frame #where does it return to/ where is it used next
    
    def grow_module(self, offset_axis, offset_axis_b, from_module_index=-1, from_stick_index=-1, face_index=0, angle=0.0, rot_angle=None):
        if rot_angle is None:
            rot_angle = self.rot_angle

        # Get position on original stick from specific module
        position = self.get_face_frame(from_module_index, from_stick_index, face_index).copy()
        
        position.point += position.xaxis * (-self.stick_length /2 - self.depth * 2)
        position.point += position.yaxis * (-self.stick_length /2)
        position.point += position.zaxis * self.depth * 2

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.zaxis, math.radians(angle), point=position.point)  #point=rotation_center
        position.transform(R)

        # Create new module at this position
        new_module = StickModule(position, self.width, self.depth, self.stick_length, rot_angle=rot_angle)
        new_module.create_module_c()
        self.modules.append(new_module)
    
    def grow_module_new(self, offset_axis, offset_axis_b, from_module_index=-1, from_stick_index=-1, face_index=0, angle=0.0, rot_angle=None):
        if rot_angle is None:
            rot_angle = self.rot_angle

        # Get position on original stick from specific module
        position = self.get_face_frame(from_module_index, from_stick_index, face_index).copy()
        
        position.point += position.xaxis * (-self.stick_length /2 + self.depth * 4)
        position.point += position.yaxis * (-self.stick_length /2 - self.depth * 4)
        position.point += position.zaxis * self.depth * 2

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.zaxis, math.radians(angle), point=position.point)  #point=rotation_center
        position.transform(R)

        # Create new module at this position
        new_module = StickModule(position, self.width, self.depth, self.stick_length, rot_angle=rot_angle)
        new_module.create_module_c_new()
        self.modules.append(new_module)

    def visualize(self):
        """
        Returns all stick geometries from all modules.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for module in self.modules for stick in module.sticks]

class BranchStickModules:
    def __init__(self, root_frame, stick_length = None, width=None, depth=None):
        self.root_frame = root_frame
        self.modules = []
        self.stick_length = stick_length
        self.width = width or  Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self._init_first_module(root_frame)
    
    def _init_first_module(self, frame):
        pt = frame.point
        # my_module = StickModuleC(pt, stick_width, stick_depth, stick_length)
        my_module = StickModule(frame, self.width, self.depth, self.stick_length)
        my_module.create_module_c() 
        self.modules.append(my_module)
    
    def get_face_frame(self, module_index, stick_index, face_index):
        module = self.modules[module_index]

        stick_frame = module.sticks[stick_index].frame

        angle = face_index*math.pi/2 # research, 0, 1, 2,3 is 90 degree steps
        R = Rotation.from_axis_and_angle(stick_frame.yaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)

        new_frame.point = module.sticks[stick_index].axis.end

        new_frame.point += new_frame.yaxis * self.depth/2
        
        return new_frame #where does it return to/ where is it used next
    
    def grow_module(self,  offset_axis, offset_axis_b, from_module_index=-1, from_stick_index=-1, face_index=0, angle=0.0):
        """
        Grows a new module from an existing module's stick.

        Args:
        from_module_index: Index of module to grow from
        from_stick_index: Index of stick within that module to grow from 
        face_index: Index of the face to grow from (0-3)
        angle: Angle of rotation in radians
        """
        
        # Get position on original stick from specific module
        position = self.get_face_frame(from_module_index, from_stick_index, face_index).copy()
        
        # rotation_center = position.point.copy()
        
        position.point += position.yaxis * self.depth/2 * offset_axis_b - position.xaxis*self.width/2
        position.point += position.xaxis * offset_axis

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)  #point=rotation_center
        position.transform(R)

        # Offset along stick axis (x, length)
        position.point += position.xaxis * offset_axis

        # Create new module at this position
        new_module = StickModule(position, self.width, self.depth, self.stick_length)
        new_module.create_module_c()
        self.modules.append(new_module)
    
    def visualize(self):
        """
        Returns all stick geometries from all modules.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for module in self.modules for stick in module.sticks]


class StickModule:
    def __init__(self, frame, stick_width, stick_depth, stick_length, rot_angle=0):
        self.frame = frame
        self.width = stick_width
        self.depth = stick_depth
        self.length = stick_length
        self.rot_angle = rot_angle
        self.sticks = []
    
    def create_module_c(self, rot_angle=None):
        x_direction = self.frame.xaxis 
        y_direction = self.frame.yaxis 
        z_direction = self.frame.zaxis 

        # Stick XA #
        offsetpt_xa = (self.frame.point
                      + x_direction * (self.depth * 0)
                      + y_direction * (self.depth * 0)
                      + z_direction * (self.depth * 0))
         
        stick_xa_axis = Line(offsetpt_xa, offsetpt_xa + x_direction * self.length)
        stick_xa = Stick(stick_xa_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_xa)

        # Stick XB #
        offsetpt_xb = (self.frame.point
                      + x_direction * (self.depth * 0)
                      + y_direction * (self.depth * 2)
                      + z_direction * (self.depth * 0))
         
        stick_xb_axis = Line(offsetpt_xb, offsetpt_xb + x_direction * self.length)
        stick_xb = Stick(stick_xb_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_xb)

        # Stick YA #
        offsetpt_ya = (self.frame.point
                      + x_direction * (self.depth * 4)
                      - y_direction * (self.depth * 4)
                      - z_direction * (self.depth * 1))
         
        stick_ya_axis = Line(offsetpt_ya, offsetpt_ya + y_direction * self.length)
        stick_ya = Stick(stick_ya_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_ya)

        # Stick YB #
        offsetpt_yb = (self.frame.point
                      + x_direction * (self.depth * 4)
                      - y_direction * (self.depth * 4)
                      + z_direction * (self.depth * 1))
         
        stick_yb_axis = Line(offsetpt_yb, offsetpt_yb + y_direction * self.length)
        stick_yb = Stick(stick_yb_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_yb)

        # Stick Z #
        offsetpt_yb = (self.frame.point
                      + x_direction * (self.length - (self.depth * 4))
                      + y_direction * (self.depth * 1)
                      - z_direction * (self.depth * 4))
         
        stick_yb_axis = Line(offsetpt_yb, offsetpt_yb + z_direction * self.length)
        stick_yb = Stick(stick_yb_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_yb)


    def create_module_c_new(self, rot_angle=None):
        x_direction = self.frame.xaxis 
        y_direction = self.frame.yaxis 
        z_direction = self.frame.zaxis 

        # Stick XA #
        offsetpt_xa = (self.frame.point
                      + x_direction * (self.depth * 0)
                      + y_direction * (self.depth * 0)
                      + z_direction * (self.depth * 0))
         
        stick_xa_axis = Line(offsetpt_xa, offsetpt_xa + x_direction * self.length)
        stick_xa = Stick(stick_xa_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_xa)

        # # Stick XB #
        # offsetpt_xb = (self.frame.point
        #               + x_direction * (self.depth * 0)
        #               + y_direction * (self.depth * 2)
        #               + z_direction * (self.depth * 0))
         
        # stick_xb_axis = Line(offsetpt_xb, offsetpt_xb + x_direction * self.length)
        # stick_xb = Stick(stick_xb_axis, self.frame, width=self.width, depth=self.depth)
        
        # self.sticks.append(stick_xb)

        # Stick YA #
        offsetpt_ya = (self.frame.point
                      + x_direction * (self.depth * 2)
                      - y_direction * (self.depth * 1.5)
                      - z_direction * (self.depth * 1))
         
        stick_ya_axis = Line(offsetpt_ya, offsetpt_ya + y_direction * self.length)
        stick_ya = Stick(stick_ya_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_ya)

        # # Stick YB #
        # offsetpt_yb = (self.frame.point
        #               + x_direction * (self.depth * 2)
        #               - y_direction * (self.depth * 2)
        #               + z_direction * (self.depth * 1))
         
        # stick_yb_axis = Line(offsetpt_yb, offsetpt_yb + y_direction * self.length)
        # stick_yb = Stick(stick_yb_axis, self.frame, width=self.width, depth=self.depth)
        
        # self.sticks.append(stick_yb)

        # Stick Z #
        offsetpt_z = (self.frame.point
                      + x_direction * (self.length - (self.depth * 2))
                      - y_direction * (self.depth * 1)
                      - z_direction * (self.depth * 2))
         
        stick_z_axis = Line(offsetpt_z, offsetpt_z + z_direction * self.length)
        stick_z = Stick(stick_z_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_z)


    def create_module_a(self, rot_angle=None):
        x_direction = self.frame.xaxis 
        y_direction = self.frame.yaxis 
        z_direction = self.frame.zaxis 

        # Stick XA #
        offsetpt_xa = (self.frame.point
                      + x_direction * (self.depth * 0)
                      + y_direction * (self.depth * 0)
                      + z_direction * (self.depth * 0))
         
        stick_xa_axis = Line(offsetpt_xa, offsetpt_xa + x_direction * self.length)
        stick_xa = Stick(stick_xa_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_xa)
        

        # Stick Z #
        offsetpt_yb = (self.frame.point
                      + x_direction * (self.length - (self.depth * 4))
                      + y_direction * (self.depth * 1)
                      - z_direction * (self.depth * 4))
         
        stick_yb_axis = Line(offsetpt_yb, offsetpt_yb + z_direction * self.length)
        stick_yb = Stick(stick_yb_axis, self.frame, width=self.width, depth=self.depth)
        
        self.sticks.append(stick_yb)
     

    def create_module_c_fridolin(self, rot_angle=None):
        ## STICK XA ##
        offsetpt_xa = (self.frame.point - self.frame.xaxis*self.width*3.5 - self.frame.yaxis*self.width)
        stick_xa = Stick(Line(offsetpt_xa, offsetpt_xa+self.frame.xaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_xa)
        
        ## STICK XB ##
        offsetpt_xb = (self.frame.point - self.frame.xaxis*self.width*3.5 + self.frame.yaxis*self.width)
        stick_xb = Stick(Line(offsetpt_xb, offsetpt_xb+self.frame.xaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_xb)
        
        ## STICK YA ##
        offsetpt_ya = (self.frame.point - self.frame.yaxis*self.width*4.5-self.frame.zaxis*self.depth)
        stick_ya = Stick(Line(offsetpt_ya, offsetpt_ya + self.frame.yaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_ya)

        ## STICK YB ##
        offsetpt_yb = (self.frame.point - self.frame.yaxis*self.width*4.5+self.frame.zaxis*self.depth)
        stick_yb = Stick(Line(offsetpt_yb, offsetpt_yb+self.frame.yaxis*self.length), width = self.width, depth = self.depth)
        self.sticks.append(stick_yb)

        ## STICK Z ##
        offsetpt_z = (self.frame.point + self.frame.xaxis*self.length-self.frame.xaxis*self.width*7 - self.frame.zaxis*self.width*3.5)
        stick_z = Stick(Line(offsetpt_z, offsetpt_z+self.frame.zaxis*self.length), width = self.width, depth = self.depth)
        
        # if rot_angle is None:
        #     rot_angle = self.rot_angle

        # # Rx = Rotation.from_axis_and_angle(self.frame.xaxis, self.rot_angle, point=self.frame.point)         
        # Ry = Rotation.from_axis_and_angle(self.frame.yaxis, rot_angle, point=self.frame.point)
        # # stick_z.geometry.transform(Rx)
        # stick_z.geometry.transform(Ry)
        
        
        self.sticks.append(stick_z)