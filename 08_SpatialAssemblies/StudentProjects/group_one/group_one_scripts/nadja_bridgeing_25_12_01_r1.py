from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box
from group_one_sticks import Stick
from nadja_studies_25_12_01_r1 import StickModule
import math



class Bridge:
    
    def __init__(self, stick_module):
        self.stick_module = stick_module
        self.sticks = [stick_module]
    

    def get_face_frame(self, module_index, face_index):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            module_index: Index of the stick
            face_index: Face index (0-3) around the stick 
        Returns:
            Frame on the specified face
        """        
        # Rotate stick frame based on index 
        # stick_frame = self.sticks[module_index].frame  
        # angle = face_index * math.pi/2   # 0--0 deg 1--90 deg 2--180 deg 3--270 deg
        # R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        # new_frame = stick_frame.transformed(R)
        # new_frame.point = self.sticks[module_index].axis.end # (get line of stick).end
        # # Offset frame to be on surface on stick
        # new_frame.point += new_frame.yaxis * (self.depth / 2) # (move along y axis)

        return self.sticks[module_index]
    
    

