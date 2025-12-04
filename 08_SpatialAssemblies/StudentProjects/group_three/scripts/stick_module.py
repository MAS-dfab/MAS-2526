from compas.geometry import Line, Frame, Vector, Rotation, Transformation, Translation
from Sticks import Stick
import math


class OStickModule:

    def __init__(self, pt, stick_length, stick_width, stick_depth, stick_offset):
        self.pt = pt  #grid point
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth
        self.offset = stick_offset
        self.sticks = []

    def create_orthogonal_module(self, type = {"x":0, "y":0, "z":0}):   # type 0: positive dir, 1: negative dir, 2: no stick
        
        #stick x
        offsetpt_x = (self.pt 
                      - Vector((self.depth/2+self.offset), 0, 0)  # move stick to be centered
                      + Vector(0, 2*self.depth*type["x"], 0)) # move stick in Y dircection based on type x = 0(positive)

        stick_x = Stick(Line(offsetpt_x, offsetpt_x+Vector(self.length, 0, 0)), width=self.width, depth=self.depth)

        if type["x"] != 2:
            self.sticks.append(stick_x)

        #stick y
        offsetpt_y = (self.pt 
                      - Vector(0, self.depth/2+self.offset, 0) # move stick to be centered
                      + Vector(0, 0, self.depth) # move stick in Z direction to avoid intersection
                      + Vector(2*self.depth*type["y"], 0, 0)) # move stick in X dircection based on type y = 0(positive)
     
        stick_y = Stick(Line(offsetpt_y, offsetpt_y+Vector(0, self.length, 0)), Vector(1,0,0), self.width, self.depth)
        #Vector(1,0,0) to define the stick's frame orientation( y or z ), here we use y direction as z_vector to avoid intersection

        if type["y"] != 2:
            self.sticks.append(stick_y)

        #stick z
        offsetpt_z = (self.pt 
                      + Vector(0, self.depth,0)
                      + Vector(self.depth,0,0)
                      - Vector(0, 0, self.depth/2+self.offset)
                      - Vector(0, 2*self.depth * type["z"],0))
       
        stick_z = Stick(Line(offsetpt_z, offsetpt_z+Vector(0, 0, self.length)), width = self.width, depth =self.depth)

        if type["z"] != 2:
            self.sticks.append(stick_z)

    def transform(self, T):
        for stick in self.sticks:
            stick.frame.transform(T)
            stick.axis.transform(T) 
    

def attach_module(base_module, base_stick_index, template_module, template_stick_index):
    import copy
    new_module = copy.deepcopy(template_module)
    new_module = template_module

    # 2. 取出要對齊的兩根 stick
    sA = base_module.sticks[base_stick_index]
    sB = new_module.sticks[template_stick_index]

    # 3. 為這兩根 stick 定義對齊用的 frame
    frA = Frame(sA.axis.end, sA.axis.direction, sA.frame.yaxis)
    frB = Frame(sB.axis.start, sB.axis.direction, sB.frame.yaxis)

    # 4. 算 from_frame_to_frame 變換
    T = Transformation.from_frame_to_frame(frB, frA)

    # 5. 把整個 new_module 丟過去
    new_module.transform(T)

    # 6. 回傳 new_module，讓你加入 assembly
    return new_module


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
        stick_axis = Line.from_point_and_vector(frame.point, frame.zaxis * self.stick_length)
        
        # Create stick 
        my_stick = Stick(stick_axis, frame.yaxis, self.width, self.depth)

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
        angle = face_index * math.pi/2   # 0--0 deg 1--90 deg 2--180 deg 3--270 deg

        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)
        new_frame.point = self.sticks[stick_index].axis.end # (get line of stick).end

        # Offset frame to be on surface on stick
        new_frame.point += new_frame.yaxis * (self.depth / 2) # (move along y axis)
        return new_frame

    def grow_stick(self, from_stick_index = -1, face_index = 0, angle = 0.0):

        """
        Grows a new stick from an existing stick.      
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in radians
        """               
        angle = math.radians(angle)

        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * (self.depth / 2)  # Offset to be outside stick
        position.point += position.xaxis * -25 # move the direction of the last stick
        print("position before rotation:", position.point)

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, angle, position.point)
        position.transform(R)                

        # Offset along stick length
        position.point += position.xaxis * -25 # move the direction of the current stick
        print("position after rotation:", position.point)

        # Create new stick
        centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector, self.width, self.depth)
        self.sticks.append(new_stick)

    def visualize(self):
        """
        Returns all stick geometries.       
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
    


#  UnitModule_THL
class UnitModule:
        def __init__(self, frame, stick_length, width = None, depth = None):
            self.frame = frame
            self.length = stick_length
            self.width = width or Stick.WIDTH
            self.depth = depth or Stick.DEPTH
            self.vertices = self._make_vertices()
            self.sticks = self._make_sticks()
        
        def _make_vertices(self):
            half = self.length / 2

            boxpts = {
                1:(-half, - half, half),
                2:(half, - half, half),
                3:(half, half, half),
                4:(-half, half, half),
                5:(-half, - half, -half),
                6:(half, - half, -half),
                7:(half, half, -half),
                8:(-half, half, -half),
            }

            vertices = {}
            origin = self.frame.point
            xaxis = self.frame.xaxis
            yaxis = self.frame.yaxis
            zaxis = self.frame.zaxis

            for i, (x, y, z) in boxpts.items():
                pt = (origin 
                      + xaxis * x
                      + yaxis * y
                      + zaxis * z)
                vertices[i] = pt
            
            return vertices
        
        def _make_sticks(self):

            pick = [1,2,3,7,8,5]
            sticks = []

            for i in range(len(pick)-1):
                a = self.vertices[pick[i]]
                b = self.vertices[pick[i+1]]
                axis = Line(a,b)

                s = Stick(axis, width=self.width, depth=self.depth)
                sticks.append(s)
            return sticks
        
        def rotated(self, angle, rotation_axis=None, pt=None):
            rotation_axis = rotation_axis or self.frame.zaxis
            pt = pt or self.frame.point
            R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), pt)
            
            new_frame = self.frame.transformed(R)
            return UnitModule(new_frame, self.length, self.width, self.depth)
        
        @classmethod
        def from_mid_stick(cls, stick, width=None, depth=None, angle = 0):
             frame = cls._frame_from_mid_stick(stick, angle)
             return cls(frame, stick.length, width, depth)
        
        @staticmethod
        def _frame_from_mid_stick(stick, angle=0):
            A = stick.start
            B = stick.end
            x = Vector.from_start_end(A, B)
            x.unitize()
            temp_vecter = Vector(0,0,1)
            if abs(temp_vecter.dot(x)) > 0.99:
                temp_vecter = Vector(1,0,0)
            
            y = temp_vecter.cross(x)
            y.unitize()

            if angle:
                R = Rotation.from_axis_and_angle(x, math.radians(angle), A)
                y = y.transformed(R)
            
            z = x.cross(y)
            z.unitize()
            half = stick.length / 2

            center = A + x * half + y * half - z * half

            return Frame(center, x, y)
        

        @classmethod
        def from_sitck_with_index(cls, stick, face_index=0, width=None, depth=None):
            frame = cls._frame_from_stick_index(stick, face_index)
            return cls(frame, stick.length, width, depth)
        
        @staticmethod
        def _frame_from_stick_index(stick, face_index):
            A = stick.start
            B = stick.end
            x = Vector.from_start_end(A, B)
            x.unitize()

            temp_vecter = Vector(0,0,1)
            if abs(temp_vecter.dot(x)) > 0.99:
                temp_vecter = Vector(1,0,0)
            y = temp_vecter.cross(x)
            y.unitize()

            angle = (face_index % 4) * 90
            if angle!= 0:
                R = Rotation.from_axis_and_angle(x, math.radians(angle), A)
                y = y.transform(R)

            z = x.cross(y)
            z.unitize()

            half = stick.length / 2
            center = A + x * half + y * half - z * half
            return Frame(center, x, y)
            


        
        
        
def connected_unit_middle(stick_a, stick_b, width=None, depth=None):
            """
            Creates a unit module connecting the midpoints of two sticks.
            Args:
                stick_a: First stick
                stick_b: Second stick
                width: Width of sticks (defaults to Stick.WIDTH)
                depth: Depth of sticks (defaults to Stick.DEPTH)
            Returns:
                UnitModule connecting the midpoints of the two sticks
            """
            p0 = stick_a.axis.midpoint
            p1 = stick_b.axis.midpoint

            v = p1 - p0
            if v.length < 0.001:
                mid0 = stick0.axis.midpoint
                mid1 = stick1.axis.midpoint
                return Stick(Line(mid0, mid1))
            else:
                return Stick(Line(p0, p1))