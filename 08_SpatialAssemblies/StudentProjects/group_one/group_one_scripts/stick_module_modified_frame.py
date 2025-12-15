from compas.geometry import Line, Frame, Vector

from group_one_sticks import Stick
import math

from compas.geometry import Rotation

class OStickModule:
    def __init__(self, pt, stick_length, stick_width, stick_depth):
        self.pt = pt
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth

        self.sticks = []

    def create_orthogonal_module(self, offset, type = {"x":0, "y":0, "z":0}):

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

class BranchingModule:
    def __init__(self, root_frame, stick_length=None, width=None, depth=None):
        """
        Constructor for Branching module.
        
        Args:
            root_frame: Frame from which tree will grow
            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """
        self.root_frame = root_frame
        self.sticks = []
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH  
        self._init_first_stick(root_frame)

    def _init_first_stick(self, frame):
        """
        Private method for creating the first stick.
        
        Args:
            frame: Frame from which stick will grow
        """
        # Draw line based on start frame
        stick_axis = Line.from_point_and_vector(frame.point, frame.zaxis*self.stick_length)
        # Create stick 
        my_stick = Stick(stick_axis, z_vector=frame.yaxis)
        # Add stick to list of sticks
        self.sticks.append(my_stick)

    def get_face_frame(self, stick_index, face_index):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            stick_index: Index of the stick
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face
        """

        # Rotate stick frame based on index 
        stick_frame = self.sticks[stick_index].frame
        angle = face_index*math.pi/2 #math.pi = 90 degrees, 0 -- 0 1--90 deg 2 --180

        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)
        new_frame.point = self.sticks[stick_index].axis.end #(get line of stick).end
    
    
        # Offset frame to be on surface on stick
        new_frame.point += new_frame.yaxis * self.depth/2 # move along y axis

        return new_frame

    def grow_stick(self, from_stick_index = -1, face_index = 0, angle = 0.0):
        """
        Grows a new stick from an existing stick.
        
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in radians
        """
                
        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy() #.copy copies frame or spell later transformED
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -10.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)
        position.transform(R)

        # Offset along stick axis (x, length)
        position.point += position.xaxis * -10.0

        # Create new stick
        centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector)
        self.sticks.append(new_stick)

    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
    
class BranchStickModules:
    def __init__(self, root_frame, stick_length = None, width=None, depth=None, z_vector = None, rot_angle = 0 ):
        
        if z_vector is None:
            self.z_vector = root_frame.zaxis
        else:
            self.z_vector = z_vector
        self.root_frame = root_frame
        self.z_vector = z_vector
        self.modules = []
        self.stick_length = stick_length
        self.width = width or  Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.rot_angle = rot_angle 
        self._init_first_module(root_frame)
    
    def _init_first_module(self, frame):
        pt = frame.point
        # my_module = StickModuleC(pt, stick_width, stick_depth, stick_length)
        my_module = StickModuleC(self.root_frame, self.width, self.depth, self.stick_length, z_vector=self.z_vector)
        my_module.create_module_c() #BIG QUESTION MARK
        self.modules.append(my_module)
    
    def get_face_frame(self, module_index, stick_index, face_index, move=1):
        module = self.modules[module_index]

        stick_frame = module.sticks[stick_index].frame

        angle = face_index*math.pi/2 # research, 0, 1, 2,3 is 90 degree steps
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)

        new_frame.point = module.sticks[stick_index].axis.end

        new_frame.point += new_frame.yaxis * -self.depth*move  #adjust Frame outputposition here 
        new_frame.point += new_frame.xaxis*-self.width*0.5 #adjust Stick overlapping
        return new_frame #where does it return to/ where is it used next
    
    def grow_module(self,  offset_xxis, offset_yxis, offset_zxis, from_module_index=-1, from_stick_index=-1, face_index=0, angle=0.0, move = 1):
        """
        Grows a new module from an existing module's stick.

        Args:
        from_module_index: Index of module to grow from
        from_stick_index: Index of stick within that module to grow from 
        face_index: Index of the face to grow from (0-3)
        angle: Angle of rotation in radians
        """
        
        # Get position on original stick from specific module
        position = self.get_face_frame(from_module_index, from_stick_index, face_index, move).copy()
        
        # rotation_center = position.point.copy()
        
        position.point += position.yaxis * self.depth * offset_yxis # is okay
        # position.point += position.xaxis * self.width * offset_axis
        position.point += position.zaxis *self.depth 
        position.point += position.zaxis * self.depth * offset_zxis

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)  #point=rotation_center
        position.transform(R)

        # Offset along stick axis (x, length)
        position.point += position.xaxis * self.depth*  offset_xxis
        # position.point += position.xaxis 
        # Create new module at this position
        new_module = StickModuleC(position, self.width, self.depth, self.stick_length)
        new_module.create_module_c()
        self.modules.append(new_module)
    
    
    def visualize(self):
        """
        Returns all stick geometries from all modules.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for module in self.modules for stick in module.sticks]

class StickModuleB:
    def __init__(self, frame, stick_width, stick_depth, stick_length):
        self.frame = frame
        self.width = stick_width
        self.depth = stick_depth
        self.length = stick_length

        self.sticks = []
    
    def create_module_b(self):
        # move stick in x
        #offsetpt_xa = (self.pt + Vector(self.width/2, 0, 0)+ Vector(0, self.width*3.5,0)+Vector(0,0,self.depth))
        offsetpt_xa = (self.frame.point + self.frame.xaxis*self.width/2 + self.frame.yaxis*self.width*3.5 + self.frame.zaxis*self.depth)
        offsetpt_xb = (self.pt + Vector(self.width/2, 0, 0)+ Vector(0, self.width*5.5,0)+Vector(0,0,self.depth))
        offsetpt_ya = (self.pt + Vector(self.width*3.5, 0, 0) - Vector(0, self.width/2,0))
        offsetpt_yb = (self.pt + Vector(self.width*3.5, 0, 0) - Vector(0, self.width/2,0) + Vector(0, 0, self.depth*2))

        stick_xa = Stick(Line(offsetpt_xa, offsetpt_xa+Vector(self.length, 0, 0)), width = self.width, depth = self.depth)
        self.sticks.append(stick_xa)

        stick_xb = Stick(Line(offsetpt_xb, offsetpt_xb+Vector(self.length, 0, 0)), width = self.width, depth = self.depth)
        self.sticks.append(stick_xb)

        stick_ya = Stick(Line(offsetpt_ya, offsetpt_ya+Vector( 0, self.length, 0)), width = self.width, depth = self.depth)
        self.sticks.append(stick_ya)

        stick_yb = Stick(Line(offsetpt_yb, offsetpt_yb+Vector( 0, self.length, 0)), width = self.width, depth = self.depth)
        self.sticks.append(stick_yb)

class StickModuleC:
    def __init__(self, root_frame, stick_width, stick_depth, stick_length, z_vector=None):
        self.root_frame = root_frame
        self.z_vector = z_vector
        self.width = stick_width
        self.depth = stick_depth
        self.length = stick_length

        self.sticks = []
    
    def create_module_c(self):
        # move stick in x
        offsetpt_xa = (self.root_frame.point - self.root_frame.xaxis*self.width*1.5 - self.root_frame.yaxis*self.width)
        offsetpt_xb = (self.root_frame.point - self.root_frame.xaxis*self.width*1.5 + self.root_frame.yaxis*self.width)
        
        offsetpt_ya = (self.root_frame.point - self.root_frame.yaxis*self.width*2.5-self.root_frame.zaxis*self.depth)
        offsetpt_yb = (self.root_frame.point - self.root_frame.yaxis*self.width*2.5+self.root_frame.zaxis*self.depth)
    
        offsetpt_z = (self.root_frame.point + self.root_frame.xaxis*self.length-self.root_frame.xaxis*self.width*3 - self.root_frame.zaxis*self.width*1.5)
        
        # Sticks along X-axis: use frame's Z-axis as normal
        stick_xa = Stick(Line(offsetpt_xa, offsetpt_xa+self.root_frame.xaxis*self.length), 
                        z_vector=self.root_frame.zaxis, width=self.width, depth=self.depth)
        self.sticks.append(stick_xa)
        
        stick_xb = Stick(Line(offsetpt_xb, offsetpt_xb+self.root_frame.xaxis*self.length), 
                        z_vector=self.root_frame.zaxis, width=self.width, depth=self.depth)
        self.sticks.append(stick_xb)
        
        # Sticks along Y-axis: use frame's Z-axis as normal
        stick_ya = Stick(Line(offsetpt_ya, offsetpt_ya + self.root_frame.yaxis*self.length), 
                        z_vector=self.root_frame.zaxis, width=self.width, depth=self.depth)
        self.sticks.append(stick_ya)

        stick_yb = Stick(Line(offsetpt_yb, offsetpt_yb+self.root_frame.yaxis*self.length), 
                        z_vector=self.root_frame.zaxis, width=self.width, depth=self.depth)
        self.sticks.append(stick_yb)

        # Stick along Z-axis: use frame's X-axis as normal (or Y-axis, depending on desired orientation)
        stick_z = Stick(Line(offsetpt_z, offsetpt_z+self.root_frame.zaxis*self.length), 
                        z_vector=self.root_frame.xaxis, width=self.width, depth=self.depth)
        self.sticks.append(stick_z)