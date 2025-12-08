from compas.geometry import Line
from compas.geometry import Rotation
import math

from Sticks import Stick

class RootModule:
    def __init__(self, branches, segment_index, stick_length=None, width=None, depth=None):
        """
        Constructor for Root module.
        
        Args:
            branches: list of frames from existing tree
            segment_index = index of frame for new root

            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """

        self.branch_frames = [b.frame for b in branches]
        self.segment_index = segment_index
        self.sticks = []

        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.segment_frame = self.get_segment_frame()

    def get_segment_frame(self):
        #get segment_frame based on segment_index
        frame = self.branch_frames[self.segment_index].copy()

        return frame

    def get_root_frame(self, face_index, rotation_angle = 0.0, segment_offset = 0.0):
        """
        Create root frame
        Translate root frame on one of the four faces of a Stick
        Create stick from frame

        Args:
            frames: List of frames
            Index: Index of frame for root frame
            face_index: Face index (0-3) around the stick

        Returns:
        Frame on the specified face
        """

        """translate root axis to be adjacent to selected segment"""

        #Rotate stick frame based on index
        face_angle = face_index * math.pi/2
        R_face = Rotation.from_axis_and_angle(self.segment_frame.xaxis, face_angle, self.segment_frame.point)
        translated_frame = self.segment_frame.transformed(R_face)
        #translate frame to end of self.segment_frame
        translated_frame.point += translated_frame.xaxis * self.stick_length/2

        #Offset frame to be on surface on stick
        translated_frame.point += translated_frame.yaxis * self.depth

        #Offset along segment axis
        translated_frame.point += -translated_frame.xaxis * segment_offset

        #Rotate along face frame
        R_stick = Rotation.from_axis_and_angle(translated_frame.yaxis, math.radians(rotation_angle), point = translated_frame.point)
        translated_frame.transform(R_stick)

        return translated_frame

    def get_root_stick(self, root_frame, root_offset = 0.0):

        # Offset along root axis
        root_frame.point += -root_frame.xaxis * root_offset

        #Create translated axis
        translated_axis = Line.from_point_and_vector(root_frame.point, root_frame.xaxis * self.stick_length)

        #create root stick using root axis
        root_stick = Stick(translated_axis, z_vector = root_frame.yaxis)

        self.sticks.append(root_stick)

    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]