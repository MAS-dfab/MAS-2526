## IMPORT BUTUH LIBRARY APA ##
from compas.geometry import Line, Vector
from group_one_sticks import Stick
from compas.geometry import Rotation, Translation
import math

##    BIKIN CLASS BUAT BIKIN UP TO 3 STICKS DARI 1 POINT    ##
class OStickModule:
    def __init__(self, frame, stick_length, stick_width, stick_depth):
        self.frame = frame
        self.pt = frame.point
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth

        self.sticks = []

        self.connection_point = None
        self.connection_point2 = None 

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
                    - Vector(0, 0, self.depth/2)      # geser dikit di z
                    + Vector(0, 2 * self.depth * type["d"], 0))  # kalau mau bisa dipakai buat offset antar stick

        stick_d = Stick(
            Line(offsetpt_d, offsetpt_d + Vector(0, 0, self.length)),
            width=self.width,
            depth=self.depth)

        if type["d"] != 2:
            self.sticks.append(stick_d)

    def create_module_1(self, type={"a": 0, "b": 0, "c": 0}):
        index_a = None

        x_direction = self.frame.xaxis 
        y_direction = self.frame.yaxis 
        z_direction = self.frame.zaxis 

        # Stick A
        offsetpt_a = (self.pt
                      - x_direction * (self.depth/2)
                      + y_direction * (1 * self.depth * type["a"]))
         
        stick_a_axis = Line(offsetpt_a, offsetpt_a + x_direction * self.length)
        stick_a = Stick(stick_a_axis, width=self.width, depth=self.depth)
        
        if type["a"] != 2:
            self.sticks.append(stick_a)
            index_a = len(self.sticks) - 1 

        # Stick A2
        offsetpt_a2 = (self.pt
                       - x_direction * (self.depth/2)
                       + y_direction * (self.depth * 2)
                       + y_direction * (1 * self.depth * type["a"]))

        stick_a2_axis = Line(offsetpt_a2, offsetpt_a2 + x_direction * self.length)
        stick_a2 = Stick(stick_a2_axis, width=self.width, depth=self.depth)
        
        if type["a"] != 2:
            self.sticks.append(stick_a2)


        # Stick B
        offsetpt_b = (self.pt 
                      - y_direction * (self.depth/2) 
                      + z_direction * (self.depth)
                      + x_direction * (self.depth)
                      - y_direction * (self.depth)
                      + x_direction * (2 * self.depth * type["b"]))

        stick_b_axis = Line(offsetpt_b, offsetpt_b + y_direction * self.length)
        stick_b = Stick(stick_b_axis, width=self.width, depth=self.depth) 

        if type["b"] != 2:
            self.sticks.append(stick_b)


        # Stick B2
        offsetpt_b2 = (self.pt 
                       - y_direction * (self.depth/2) 
                       + z_direction * (self.depth)
                       + x_direction * (self.depth * 3)
                       - y_direction * (self.depth)
                       + x_direction * (2 * self.depth * type["b"]))
        
        stick_b2_axis = Line(offsetpt_b2, offsetpt_b2 + y_direction * self.length)
        stick_b2 = Stick(stick_b2_axis, width=self.width, depth=self.depth) 

        if type["b"] != 2:
            self.sticks.append(stick_b2)


        # Stick C 
        offsetpt_c = (self.pt 
                      + y_direction * (self.depth)
                      + x_direction * (self.depth * 2)
                      - z_direction * (self.length/2)
                      - y_direction * (2 * self.depth * type["c"]))
        
        stick_c_axis = Line(offsetpt_c, offsetpt_c + z_direction * self.length)
        stick_c = Stick(stick_c_axis, width=self.width, depth=self.depth)

        if type["c"] != 2:
            self.sticks.append(stick_c)


        # connection point #
        if index_a is not None:

            stickA = self.sticks[index_a]

            # midpoint of Stick A
            midA = stickA.axis.midpoint

            # connection_point 1 (pakai arah lokal)
            offset1 = (x_direction * (self.depth * 3) +
                       y_direction * (self.depth))
            self.connection_point = midA + offset1

            # connection_point 2
            offset2 = (x_direction * (-self.depth * 5.2) +
                       y_direction * (self.length - self.depth * 5))
            self.connection_point2 = midA + offset2

        else:
            self.connection_point = None
            self.connection_point2 = None

    def create_module_2(self, type={"a": 0, "b": 0, "c": 0}):
        index_b = None

        # Stick A
        offsetpt_a = (self.pt
                      - Vector(self.depth/2,0,0)
                      - Vector(self.depth*1.5, 0,0)
                      + Vector(0, 1* self.depth * type["a"], 0))
                      
        
        stick_a = Stick(Line(offsetpt_a, offsetpt_a + Vector(self.length, 0, 0)), width=self.width, depth=self.depth)
        
        if type["a"] != 2:
            self.sticks.append(stick_a)

        # Stick A2
        offsetpt_a2 = (self.pt
                      - Vector(self.depth/2,0,0)
                      + Vector(0, self.depth*2 ,0)
                      + Vector(self.depth*2, 0,0)
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
            index_b = len(self.sticks) - 1 

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


        # connection point #
        if index_b is not None:

            stickA = self.sticks[index_b]

            # midpoint of Stick A
            midA = stickA.axis.end

            # connection_point 1
            offset1 = Vector(-1*self.depth, -5*self.depth, 0)
            self.connection_point = midA + offset1

            # connection_point 2
            offset2 = Vector(6*self.depth, -10.9*self.depth, 0)
            self.connection_point2 = midA + offset2
        else:
            self.connection_point = None

    def create_module_3(self, type={"a": 0, "b": 0, "c": 0}):
        index_c = None
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
            index_c = len(self.sticks) - 1

        # connection point #
        if index_c is not None:

            stick_c = self.sticks[index_c]

            # midpoint of Stick C
            midC = stick_c.axis.end

            # connection_point 1
            offset1 = Vector(0,0,0)
            self.connection_point = midC + offset1

            # connection_point 2
            offset2 = Vector(0,0,-self.length)
            self.connection_point2 = midC + offset2
        else:
            self.connection_point = None


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
        angle = face_index * math.pi/2 # 0-- 0 deg 1-- 90 deg 2 -- 180 deg 3 -- 270 deg

        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle=angle, point=stick_frame.point)
        new_frame = stick_frame.transformed(R) 

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

    def _get_cp(self, module, port):
        """Return connection_point or connection_point2 based on port=1 or 2."""
        if port == 1:
            return getattr(module, "connection_point", None)
        elif port == 2:
            return getattr(module, "connection_point2", None)
        else:
            return None

    def build_module(self, pt, mode, type_dictionary):
        module = OStickModule(pt, self.length, self.width, self.depth)
        function_name = self.module_funcations[mode]
        function = getattr(module, function_name)
        function(type_dictionary)

        return module
    
    def add_root_module(self, pt, mode, type_dictionary):
        root = self.build_module(pt, mode, type_dictionary)
        self.modules.append(root)
        return len(self.modules)-1
    
    def grow_module_from(self, direction, gap, base_pt, mode, type_dictionary):
        DIR = {
            "x+": Vector(1,0,0),
            "x-": Vector(-1,0,0),
            "y+": Vector(0,1,0),
            "y-": Vector(0,-1,0),
            "z+": Vector(0,0,1),
            "z-": Vector(0,0,-1),
        }

        offset = DIR[direction] * (self.length + gap)
        new_pt = base_pt + offset

        new_module = self.build_module(new_pt, mode, type_dictionary)
        self.modules.append(new_module)
        return len(self.modules) - 1

    def grow_from_connection(self, from_index, mode, type_dictionary, parent_port=1, child_port=1, angle=0.0, offset=None):
        """
        Grow a new module starting from the connection_point of an existing module.
        """
        parent_module = self.modules[from_index]
        parent_cp = self._get_cp(parent_module, parent_port)
        print("GROW call | from_index:", from_index, "| parent_port:", parent_port, "| parent_cp:", parent_cp)

        # if parent has no connection point, we can't attach
        if parent_cp is None:
            print("  -> parent_cp is None, no module created")
            return None

        # 1) build the child module at some provisional location
        #    (we'll move it later so cp_child = cp_parent + offset)
        child = self.build_module(parent_cp, mode, type_dictionary)
        child_cp = self._get_cp(child, child_port)

        # if child has no connection point, just store it and stop
        if child_cp is None:
            self.modules.append(child)
            return len(self.modules) - 1

        # 2) compute target connection point (parent_cp + offset)
        if offset is None:
            offset = Vector(0, 0, 0)

        target_cp = parent_cp + offset

        # 3) TRANSLATE CHILD so that child_cp moves to target_cp
        move_vec = target_cp - child_cp
        T = Translation.from_vector(move_vec)

        # move all sticks of the child
        for s in child.sticks:
            s.axis = s.axis.transformed(T)
            if hasattr(s, "frame") and s.frame is not None:
                s.frame = s.frame.transformed(T)

        # move module base point & connection_point
        child.pt = child.pt.transformed(T)
    
        # move both connection points (if they exist)
        if getattr(child, "connection_point", None) is not None:
            child.connection_point = child.connection_point.transformed(T)

        if getattr(child, "connection_point2", None) is not None:
            child.connection_point2 = child.connection_point2.transformed(T)

        # 4) ROTATE CHILD AROUND THE SHARED CONNECTION POINT
        if angle != 0.0:
            axis_vec = Vector(0, 0, 1)  # rotate around global Z

            for s in child.sticks:
                s.rotate_stick(angle, axis_vec, target_cp)

            R = Rotation.from_axis_and_angle(axis_vec, math.radians(angle), target_cp)

            child.pt = child.pt.transformed(R)

            if getattr(child, "connection_point", None) is not None:
                child.connection_point = child.connection_point.transformed(R)

            if getattr(child, "connection_point2", None) is not None:
                child.connection_point2 = child.connection_point2.transformed(R)


        # 5) store child
        self.modules.append(child)
        new_index = len(self.modules) - 1
        print("  -> new_index:", new_index, "| child.pt:", child.pt)
        return new_index
        
    def visualize(self):
        geoms = []
        for m in self.modules:
            for s in m.sticks:
                geoms.append(s.geometry)
        return geoms
    