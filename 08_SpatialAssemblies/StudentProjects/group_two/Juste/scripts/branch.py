from compas.geometry import Line, Frame, Vector
from compas.geometry import Rotation
import math

from Sticks import Stick

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

        #self._init_first_stick(root_frame)

    def _init_first_stick(self, frame):
        """
        Private method for creating the first stick.
        
        Args:
            frame: Frame from which stick will grow
        """
        # Draw line based on start frame
        stick_axis = Line.from_point_and_vector(frame.point, frame.zaxis * self.stick_length)

        # Create stick 
        st_stick = Stick(stick_axis, z_vector = frame.yaxis)

        # Add stick to list of sticks
        self.sticks.append(st_stick)

    def get_face_frame(self, stick_index, face_index):
        """
        Gets the frame at each face of a stick. 
        Args:
            stick_index: Index of stick to get face from
            face_index: Index of face to get frame from (0-3)
        Returns:
            Frame at the specified face
        """
        stick = self.sticks[stick_index]
        base_frame = stick.frame.copy()
        end_pt = stick.axis.end

        # Identify face direction
        if face_index == 0:
            normal = base_frame.yaxis
        elif face_index == 1:
            normal = -base_frame.yaxis
        elif face_index == 2:
            normal = base_frame.zaxis
        elif face_index == 3:
            normal = -base_frame.zaxis
        else:
            raise ValueError("face_index must be 0, 1, 2, or 3.")

        # Construct a new frame pointing in the direction of the face normal
        xaxis = base_frame.xaxis
        yaxis = normal
        zaxis = xaxis.cross(yaxis).unitized()

        face_frame = Frame(end_pt + yaxis * (self.depth / 2), xaxis, yaxis)

        return face_frame

         
    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=0.0):
        """
        Grows a new stick from a specified face of an existing stick.
        Args:
            from_stick_index: Index of stick to grow from (default: last stick)
            face_index: Index of face to grow from (0-3)
            angle: In-plane rotation angle in degrees (default: 0.0)
            offset: Offset distance along growth direction (default: 0.0)
        Returns:
            None

            """
        if not self.sticks:
            # FIRST stick — grow from root_frame directly
            axis = Line.from_point_and_vector(self.root_frame.point, self.root_frame.xaxis * self.stick_length)
            z_vector = self.root_frame.yaxis
            new_stick = Stick(axis, z_vector)
            self.sticks.append(new_stick)
            return  # skip rest of method   

        #otherwise, grow from specified stick face                            
        position = self.get_face_frame(from_stick_index, face_index).copy()

        # Shift outward another half-depth to ensure separation
        position.point += position.yaxis * (self.depth * 0.5)

        # Apply offset if needed
        position.point -= position.xaxis * offset

        # Optional in-plane rotation around Z (if any angle is needed)
        if angle != 0.0:
            R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)
            position.transform(R)

        # Create new stick along X axis from the face frame
        axis = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        z_vector = position.yaxis

        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)


    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]