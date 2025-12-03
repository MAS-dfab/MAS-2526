import math
from compas.geometry import Line, Rotation
from Sticks import Stick

from sticks import Stick
import math

from compas.geometry import Rotation


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
    """
    Branching module that grows new sticks from the faces of an existing root stick.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None):
        """
        Initialize a branching module starting from an existing Stick object.

        Parameters
        ----------
        root_stick : Stick
            The first stick of the structure. Branching begins from its faces.
        stick_length : float, optional
            Length of each stick grown in branching.
        width : float, optional
            Width of each stick (defaults to Stick.WIDTH).
        depth : float, optional
            Depth of each stick (defaults to Stick.DEPTH).
        """
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
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
        angle = face_index * math.pi/2 # 0-- 0 deg 1-- 90 deg 2-- 180 deg 3 -- 270 deg
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        new_frame = stick_frame.transformed(R)
        new_frame.point = self.sticks[stick_index].axis.end # (get line of stick).end
        # Offset frame to be on surface on stick
        new_frame.point += new_frame.yaxis * self.depth/2 # (move along y axis)

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
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -10.0

        # Rotate along face frame
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)
        position.transform(R)

        # Offset along stick length
        position.point += position.xaxis * -10.0

        # Create new stick
        centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector)
        self.sticks.append(new_stick)


    def visualize(self):
        """
        Returns geometry of all sticks as COMPAS Box objects.
        """
        return [s.geometry for s in self.sticks]
