from compas.geometry import Line, Frame, Vector, Plane, Point
from Sticks_Single_1 import Stick
import math
from compas.geometry import Rotation, Translation, Transformation


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
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH        
        self.sticks = []
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
        angle = face_index * math.pi/2 # 0: 0 deg, 1: 90 deg, 2: 180 deg, 3: 270 deg

        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R) 
        new_frame.point = self.sticks[stick_index].axis.end # (get line of stick).end

        # Offset frame to be on surface of stick
        new_frame.point += new_frame.yaxis * self.depth/2 # (move along y axis)

        return new_frame
         
    def grow_stick(self, from_stick_index = -1, face_index = 0, angle = 0.0):
        """
        Grows a new stick from an existing stick.
        
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in degree
        """
                
        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -10.0 # move the direction of the last stick

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle),point=position.point) 
        position.transform(R)

        # Offset along stick length
        position.point += position.xaxis * -10.0 # move the direction of the current stick

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
    
class TiltedModule:
    def __init__(self, base_frame, root_angle, stick_d_length = None, stick_u_length = None, width = None, depth = None):

        self.base_frame = base_frame
        self.root_angle = root_angle
        self.stick_d_length = stick_d_length
        self.stick_u_length = stick_u_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.sticks = []
        self._init_first_stick(base_frame, root_angle)
    
    def _init_first_stick(self, frame, angle):

        # Rotate the frame
        R = Rotation.from_axis_and_angle(axis = frame.zaxis, angle = math.radians(angle), point = frame.point)
        tilted_frame = frame.transformed(R)

        # Draw a line based on start frame
        stick_axis = Line.from_point_and_vector(point = tilted_frame.point, vector = tilted_frame.zaxis * self.stick_d_length)

        # Create the first stick
        # first_stick = Stick(axis = stick_axis, z_vector = tilted_frame.yaxis)
        first_stick = Stick(frame = tilted_frame, length = self.stick_d_length)


        # Add stick to list of sticks
        self.sticks.append(first_stick)
    
    def get_face_frame(self, stick_index, face_index):

        # Rotate stick frame based on index
        stick_frame = self.sticks[stick_index].frame
        angle = face_index * math.pi/2

        R = Rotation.from_axis_and_angle(axis = stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)
        new_frame.point = self.sticks[stick_index].axis.end

        # Move frame attached on the face
        new_frame.point += new_frame.yaxis * self.depth/2

        return new_frame
    

    
    def grow_stick(self, length = None, from_stick_index = -1, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -108.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -60.0

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)
    
    def grow_shorter_stick(self, length = None, from_stick_index = 0, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += Vector(0,1,0) * 13.685
        position.point += position.xaxis * -length/2

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -length/2

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)
    
    def grow_longer_stick(self, length = None, from_stick_index = -1, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -108.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -60.0

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)

    def symmetrical_stick(self, angle=180, m_distanece=50.7):

        # Build rotation origin
        rotation_origin = self.base_frame.point + Vector(0, 1, 0) * m_distanece + Vector(self.depth/2,0,0)

        # Build rotation axis (a line in Z direction)
        # rotation_axis = Line(rotation_origin, rotation_origin + Vector(0, 0, 1) * 10)
        rotation_axis = Vector(0,0,1)

        # Create rotation transform
        R = Rotation.from_axis_and_angle(
            axis=rotation_axis,
            angle=math.radians(angle),   # convert degree → radians
            point=rotation_origin
        )

        # Rotate each stick
        rotated_sticks = []
        for stick in self.sticks:

            new_frame = stick.frame.transformed(R)

            new_stick = Stick(
                frame=new_frame,
                length=stick.length,
                width=stick.width,
                depth=stick.depth
            )

            # Move toward x vector 26 unit
            M = Translation.from_vector(Vector(26,0,0))
            # new_stick.transformed(M)
            rotated_sticks.append(new_stick)

        # Add rotated sticks
        self.sticks.extend(rotated_sticks)
    
    def visualize(self):
        return [stick.geometry for stick in self.sticks]


class TiltedModule_4legs:
    def __init__(self, base_frame, root_angle, stick_d_length = None, stick_u_length = None, width = None, depth = None):

        self.base_frame = base_frame
        self.root_angle = root_angle
        self.stick_d_length = stick_d_length
        self.stick_u_length = stick_u_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.sticks = []
        self._init_first_stick(base_frame, root_angle)
    
    def _init_first_stick(self, frame, angle):

        # Rotate the frame
        R = Rotation.from_axis_and_angle(axis = frame.zaxis, angle = math.radians(angle), point = frame.point)
        tilted_frame = frame.transformed(R)

        # Draw a line based on start frame
        stick_axis = Line.from_point_and_vector(point = tilted_frame.point, vector = tilted_frame.zaxis * self.stick_d_length)

        # Create the first stick
        # first_stick = Stick(axis = stick_axis, z_vector = tilted_frame.yaxis)
        first_stick = Stick(frame = tilted_frame, length = self.stick_d_length)


        # Add stick to list of sticks
        self.sticks.append(first_stick)
    
    def get_face_frame(self, stick_index, face_index):

        # Rotate stick frame based on index
        stick_frame = self.sticks[stick_index].frame
        angle = face_index * math.pi/2

        R = Rotation.from_axis_and_angle(axis = stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)
        new_frame.point = self.sticks[stick_index].axis.end

        # Move frame attached on the face
        new_frame.point += new_frame.yaxis * self.depth/2

        return new_frame
    

    
    def grow_stick(self, length = None, from_stick_index = -1, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -108.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -60.0

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)
    
    def grow_tilted_stick(self, length = None, from_stick_index = -1, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        # position.point += position.xaxis * -120.0
        position.point += position.xaxis * -108.0


        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        # new_position.point += new_position.xaxis * -180.0
        new_position.point += new_position.xaxis * -180.0
        

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)

    
    def grow_shorter_stick(self, length = None, from_stick_index = 0, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += Vector(0,1,0) * 13.685
        position.point += position.xaxis * -length/2

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -length/2

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)
    
    def grow_longer_stick(self, length = None, from_stick_index = -1, face_index = 0, angle = 0.0):

        # get position (frame) on original stick
        position = self.get_face_frame(stick_index = from_stick_index, face_index = face_index)
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -108.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(axis = position.yaxis, angle = math.radians(angle), point = position.point)
        new_position = position.transformed(R)

        # Offset along stick length
        new_position.point += new_position.xaxis * -60.0

        # Create new stick
        new_stick = Stick(frame = new_position, length = length)

        self.sticks.append(new_stick)

    def symmetrical_stick(self, angle=180, m_distanece=50.7):

        # Build rotation origin
        rotation_origin = self.base_frame.point + Vector(0, 1, 0) * m_distanece + Vector(self.depth/2,0,0)

        # Build rotation axis (a line in Z direction)
        # rotation_axis = Line(rotation_origin, rotation_origin + Vector(0, 0, 1) * 10)
        rotation_axis = Vector(0,0,1)

        # Create rotation transform
        R = Rotation.from_axis_and_angle(
            axis=rotation_axis,
            angle=math.radians(angle),   # convert degree → radians
            point=rotation_origin
        )

        # Rotate each stick
        rotated_sticks = []
        for stick in self.sticks:

            new_frame = stick.frame.transformed(R)

            new_stick = Stick(
                frame=new_frame,
                length=stick.length,
                width=stick.width,
                depth=stick.depth
            )

            # Move toward x vector 26 unit
            M = Translation.from_vector(Vector(26,0,0))
            # new_stick.transformed(M)
            rotated_sticks.append(new_stick)

        # Add rotated sticks
        self.sticks.extend(rotated_sticks)

    def visualize(self):
        return [stick.geometry for stick in self.sticks]
