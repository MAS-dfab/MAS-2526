from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box, Transformation
from sticks_251207 import Stick
from nadja_stick_module import StickModuleA
import math


class ModuleConnection:
    
    def __init__(self, stick_module, root_frame, stick_angle, stick_angles_z):
        self.stick_module = stick_module
        self.root_frame = root_frame
        self.stick_angle = stick_angle
        self.stick_angles_z = stick_angles_z
        self.modules = [stick_module]
        self.angles = []
        self.module_frames = [root_frame]

    def get_face_frame(self, module_index, face_index, stick="stick1"):
        """
        Gets a frame on one of the four faces of a chosen stick within a module.
        Args:
            module_index: Index of the stick
            face_index: Face index (0-3) around the stick 
            stick: Stick object within the module
        Returns:
            Frame on the specified face
        """        
        # get face frames of the stick
        module = self.modules[module_index]
        stick_frame = module[stick].frame
        angle = face_index * math.pi/2   # 0--0 deg 1--90 deg 2--180 deg 3--270 deg
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        face_frame = stick_frame.transformed(R)
        # ofset the frame to be at the face of the stick
        face_frame.point += face_frame.zaxis * (module[stick].depth / 2)
        
        return face_frame

    def grow_module(self, from_module_index, from_face_index=0, from_stick="stick1", type = 0):
        """
        Grows a new module from a specified face of a chosen stick within a module.
        Args:
            from_module_index: Index of the module to grow from
            from_face_index: Face index (0-3) around the stick
            from_stick: Stick object within the module
        Returns:
            Newly created module
        """
        # connection type 1: grow module from stick3 face 0 of old module to stick1 face 0 of new module
        face_frame_to_connect = self.get_face_frame(from_module_index, 0, stick="stick3") # get the frame from base module
        face_frame_to_grow_from = self.get_face_frame(from_module_index, 0, stick="stick1") # get the frame from module index to grow from
        
        # offset the connect frame along stick axis to avoid intersection 
        if self.stick_angle[from_module_index] < 0:
            face_frame_to_connect.point += face_frame_to_connect.xaxis * 25
        else:
            face_frame_to_connect.point -= face_frame_to_connect.xaxis * 25
        
        # calculate angle between normals of the two frames 
        angle = face_frame_to_connect.zaxis.angle_signed(face_frame_to_grow_from.zaxis, face_frame_to_connect.yaxis)
        rotation_angle = (math.pi - angle)        
      
        # translation vector of the base_frame 
        v1 = Vector.from_start_end(self.module_frames[from_module_index].point, face_frame_to_connect.point)
        v2 = Vector.from_start_end(self.module_frames[from_module_index].point, face_frame_to_grow_from.point)
        translation_vector = v1 - v2
        new_frame = self.module_frames[from_module_index].translated(translation_vector)
        
        # rotate the frame to grow from to align with the connect frame
        R = Rotation.from_axis_and_angle(face_frame_to_connect.yaxis, angle=rotation_angle, point=face_frame_to_connect.point)
        rotated_new_plane = new_frame.transformed(R)
        
        # add rotation along x axis to create twist
        R_twist = Rotation.from_axis_and_angle(face_frame_to_connect.zaxis, angle=math.radians(self.stick_angles_z[from_module_index]), point=face_frame_to_connect.point)
        twisted_new_plane = rotated_new_plane.transformed(R_twist)
        
        # create new module at the new frame
        module = StickModuleA(twisted_new_plane, angle=self.stick_angle[from_module_index+1])
        new_module = module.create_module()
        modules = [module.sticks[stick] for stick in new_module]
        
        # append to the list of modules 
        self.modules.append(new_module)
        self.angles.append(math.degrees(angle))
        self.module_frames.append(twisted_new_plane)

        return modules
        
        