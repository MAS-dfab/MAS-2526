from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box
from group_one_sticks import Stick
from nadja_studies_25_12_01_r1 import StickModule
import math



class Bridge:
    
    def __init__(self, stick_module):
        self.stick_module = stick_module
        self.sticks = [stick_module]
        self.width = Stick.WIDTH
        self.depth = Stick.DEPTH 
        self.face_centroids = []

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
        # stick_frame = self.sticks[module_index][0].frame 
        # angle = face_index * math.pi/2   # 0--0 deg 1--90 deg 2--180 deg 3--270 deg
        # R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        # new_frame = stick_frame.transformed(R)
        # new_frame.point = self.sticks[module_index][0].diagonal.end # (get line of stick).end
        # # Offset frame to be on surface on stick
        # new_frame.point += new_frame.yaxis * (self.depth / 2) # (move along y axis)
        
        # frames of the faces 
        face_frames = []
        geometry = self.sticks[module_index][0].geometry
        faces = geometry.to_brep().faces
        for face in faces:
            center_point = face.centroid
            face_frame = face.frame_at(0, 0)
            center_frame = Frame(center_point, face_frame.xaxis, face_frame.yaxis)
            face_frames.append(center_frame)
            self.face_centroids.append(center_point)

        return face_frames[face_index]
    
    # @property
    # def get_face_index(self, module_index):
    #     faces = self.sticks[module_index][0].to_brep().faces
    #     face_centroids = []
    #     for face in faces:
    #         center_point = face.centroid
    #         face_centroids.append(center_point)
    #     return face_centroids

        
    def grow_module(self, module, from_stick_index = 0, face_index = 0, angle = 0.0):
        """
        Grows a new stick from an existing stick.
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in radians
        """
        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy()
        # position.point += position.yaxis * (self.depth / 2)  # Offset to be outside stick
        # position.point += position.xaxis * -10
        
        # Rotate z axis so it is parallel with the face
        R_1 = Rotation.from_axis_and_angle(position.yaxis, math.radians(90), position.point)
        R_3 = Rotation.from_axis_and_angle(position.xaxis, math.radians(-90), position.point)
        position.transform(R_1)
        position.transform(R_3)
        
        # Rotate along face frame
        R_2 = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), position.point)
        position.transform(R_2)
        position.point += position.yaxis * (-self.depth / 2)  # Offset to be outside stick
        # Offset along stick length
        # position.point += position.xaxis * -.10
        
        # Create new stick
        # centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        # zvector = position.yaxis
        # new_stick = Stick(centerline, zvector)
        # self.sticks.append(new_stick)
        
        # Add new module
        stick_module = module.CreateModule(position)
        self.sticks.append(stick_module)
        return stick_module
        
        
        
    

