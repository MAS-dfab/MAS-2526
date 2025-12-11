from compas.geometry import Line, Frame, Vector
from Sticks import Stick
from compas.geometry import Rotation
import math

class OStickModule:
    def __init__(self, pt, stick_length, stick_width, stick_depth):
        self.pt = pt
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth

        self.sticks = []
    def create_orthogonal_module(self, type = {"x": 0, "y": 0, "z": 0}):

        #stick x
        offsetpt_x = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, 2* self.depth * type["x"], 0))
        stick_x = Stick(Line(offsetpt_x,offsetpt_x + Vector(self.length,0,0)), width=self.width, depth=self.depth)
        if type["x"] !=2:
            self.sticks.append(stick_x)

        #stick y
        offsetpt_y = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(2* self.depth * type["y"],0,0))
        stick_y = Stick(Line(offsetpt_y, offsetpt_y + Vector(0,self.length,0)), width=self.width, depth=self.depth)
        if type["y"] !=2:
            self.sticks.append(stick_y)

        #stick z
        offsetpt_z= (self.pt 
                      + Vector(0,self.depth,0) 
                      + Vector (self.depth,0,0) 
                      - Vector(0,0,self.depth/2)
                      - Vector(0, 2* self.depth * type["z"],0))
        stick_z = Stick(Line(offsetpt_z, offsetpt_z + Vector(0,0,self.length)), width=self.width, depth=self.depth)
        if type["z"] !=2:
            self.sticks.append(stick_z)


    def create_triangular_module(self, type={"a": 0, "b": 0, "c": 0, "d": 2}):
        
        # Stick A
        offsetpt_a = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        stick_a = Stick(Line(offsetpt_a, offsetpt_a + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        if type["a"] != 2:
            self.sticks.append(stick_a)

        # Stick B
        offsetpt_b = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b = Stick(Line(offsetpt_b, offsetpt_b + Vector(0.5 * self.length, 0.866 * self.length, 0)), width=self.width, depth=self.depth)  # +60°
        if type["b"] != 2:
            self.sticks.append(stick_b)

        # Stick C 
        offsetpt_c = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth*2)
                      + Vector(2* self.depth * type["c"],0,0))
        stick_c = Stick(Line(offsetpt_c, offsetpt_c + Vector(-0.5 * self.length, 0.866 * self.length, 0)), width=self.width, depth=self.depth)  # 120°
        if type["c"] != 2:
            self.sticks.append(stick_c)

        # Stick D (vertical Z)
        offsetpt_d = (self.pt
                    - Vector(0, 0, self.depth/2)      
                    + Vector(0, 2 * self.depth * type["d"], 0))  
        stick_d = Stick(
            Line(offsetpt_d, offsetpt_d + Vector(0, 0, self.length, 0)),
            width=self.width,
            depth=self.depth)
        if type["d"] != 2:
            self.sticks.append(stick_d)

    def create_module_1(self, type={"a": 0, "b": 0, "c": 0}):
        
        # Stick A
        offsetpt_a = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        stick_a = Stick(Line(offsetpt_a, offsetpt_a + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        if type["a"] != 2:
            self.sticks.append(stick_a)

        # Stick A2
        offsetpt_a2 = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, self.depth*2 ,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        stick_a2 = Stick(Line(offsetpt_a2, offsetpt_a2 + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        if type["a"] != 2:
            self.sticks.append(stick_a2)

        # Stick B
        offsetpt_b = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(self.depth, 0,0)
                      - Vector(0, self.depth, 0)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b = Stick(Line(offsetpt_b, offsetpt_b + Vector(0, self.length, 0)), width=self.width, depth=self.depth) 
        if type["b"] != 2:
            self.sticks.append(stick_b)

        # Stick B2
        offsetpt_b2 = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(self.depth*3, 0,0)
                      - Vector(0, self.depth, 0)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b2 = Stick(Line(offsetpt_b2, offsetpt_b2 + Vector(0, self.length, 0)), width=self.width, depth=self.depth) 
        if type["b"] != 2:
            self.sticks.append(stick_b2)

        # Stick C 
        offsetpt_c = (self.pt 
                      + Vector(0,self.depth,0)
                      + Vector(self.depth*2,0,0)
                      - Vector(0,0,self.length/2)
                      - Vector(0, 2*self.depth * type["c"],0))
        stick_c = Stick(Line(offsetpt_c, offsetpt_c + Vector(0,0,self.length)), width = self.width,depth =self.depth)
        if type["c"] != 2:
            self.sticks.append(stick_c)


    def create_module_2(self, type={"a": 0, "b": 0, "c": 0}):
        
        # Stick A
        offsetpt_a = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        stick_a = Stick(Line(offsetpt_a, offsetpt_a + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        if type["a"] != 2:
            self.sticks.append(stick_a)

        # Stick A2
        offsetpt_a2 = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, self.depth*2 ,0)
                      + Vector(self.length/2, 0,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        stick_a2 = Stick(Line(offsetpt_a2, offsetpt_a2 + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        if type["a"] != 2:
            self.sticks.append(stick_a2)

        # Stick B
        offsetpt_b = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(self.length - self.depth*3, 0,0)
                      - Vector(0, self.depth*2, 0)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b = Stick(Line(offsetpt_b, offsetpt_b + Vector(0, self.length, 0)), width=self.width, depth=self.depth) 
        if type["b"] != 2:
            self.sticks.append(stick_b)

        # Stick B2
        offsetpt_b2 = (self.pt 
                      - Vector(0, self.depth/2,0) 
                      + Vector(0,0,self.depth)
                      + Vector(self.length - self.depth*5, 0,0)
                      - Vector(0, self.depth*2, 0)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b2 = Stick(Line(offsetpt_b2, offsetpt_b2 + Vector(0, self.length, 0)), width=self.width, depth=self.depth) 
        if type["b"] != 2:
            self.sticks.append(stick_b2)

        # Stick C 
        offsetpt_c = (self.pt 
                      + Vector(0,self.depth,0)
                      + Vector(self.length-self.depth*4,0,0)
                      - Vector(0,0,self.length/2)
                      - Vector(0, 2*self.depth * type["c"],0))
        stick_c = Stick(Line(offsetpt_c, offsetpt_c + Vector(0,0,self.length)), width = self.width,depth =self.depth)
        if type["c"] != 2:
            self.sticks.append(stick_c)


    def create_module_3(self, type={"a": 0, "b": 0, "c": 0}):
        
        # Stick A
        offsetpt_a = (self.pt
                      + Vector(self.length/2 - self.depth*2, 0, 0)
                      + Vector(0, self.depth, 0)
                      + Vector(0, 1* self.depth * type["a"], 0))
        # Desired diagonal direction
        direction = Vector(-1, 1, 0)
        # Make the direction length = 1
        unit_dir = direction.unitized()
        # Scale up to the stick length
        vec = unit_dir * self.length
        # Create the stick
        stick_a = Stick(Line(offsetpt_a, offsetpt_a + vec), width=self.width, depth=self.depth)              
        if type["a"] != 2:
            self.sticks.append(stick_a)

        # Stick B
        offsetpt_b = (self.pt 
                      - Vector(0,0,0) 
                      + Vector(0,0,self.depth)
                      - Vector(0, self.depth, 0)
                      + Vector(2* self.depth * type["b"],0,0))
        stick_b = Stick(Line(offsetpt_b, offsetpt_b + Vector(0, self.length, 0)), width=self.width, depth=self.depth) 
        if type["b"] != 2:
            self.sticks.append(stick_b)

        # Stick C 
        offsetpt_c = (self.pt 
                      + Vector(0,self.length/2 - self.depth*1.75,0)
                      - Vector(self.depth,0,0)
                      - Vector(0,0,self.length - self.depth * 5)
                      - Vector(0, 2*self.depth * type["c"],0))
        stick_c = Stick(Line(offsetpt_c, offsetpt_c + Vector(0,0,self.length)), width = self.width,depth =self.depth)
        if type["c"] != 2:
            self.sticks.append(stick_c)


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
        # Draw line based on start frame >>>>>> buat garis untuk bikin stick
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
        angle = face_index * math.pi/2 # 0-- 0 deg 1-- 90 deg 2 -- 180 deg 3 -- 270 deg

        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle=angle, point=stick_frame.point)
        new_frame = stick_frame.transformed(R) ## PASTIKAN TRANSFORMED VS TRANSFORM

        new_frame.point = self.sticks[stick_index].axis.end # (get line of stick).end

        # Offset frame to be on surface on stick
        new_frame.point += new_frame.yaxis * self.depth/2  # (move along y axis)

        return new_frame
         
    def grow_stick(self, from_stick_index = -1, face_index = 0, angle = 0.0):
    ##### -1 on python means the last stick  ######
        """
        Grows a new stick from an existing stick.
        
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in radians
        """
                
        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -10.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, angle=math.radians(angle), point=position.point)
        position.transform(R)

        # Offset along stick length
        position.point += position.xaxis * -10.0
        # Create new stick
        centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector)

        self.sticks.append(new_stick) ## JANGAN LUPA DI APPEND WKWKWK

    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
    


class Branching_Module_to_Module:
    def __init__(self, stick_length, stick_width, stick_depth):
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth
        self.modules = []

        self.module_funcations = {
            "orthogonal": "create_orthogonal_module",
            "triangular": "create_triangular_module",
            "module_1": "create_module_1",
            "module_2": "create_module_2",
            "module_3": "create_module_3",
        }

    def build_module(self, pt, mode, type_dictionary):
        module = OStickModule(pt, self.length, self.width, self.depth)
        function_name = self.module_funcations[mode]
        function = getattr(module, function_name)
        function(type_dictionary)

        return module
    
