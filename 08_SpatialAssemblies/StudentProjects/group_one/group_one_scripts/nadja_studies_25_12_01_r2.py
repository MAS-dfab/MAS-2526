from compas.geometry import Line, Frame, Vector, Rotation, Polyline, Plane, Point, Box, Transformation
from sticks_251207 import Stick
from nadja_mainmodule_25_12_04_r1 import StickModuleA
import math


class ModuleConnection:
    
    def __init__(self, stick_module, root_frame, stick_angle):
        self.stick_module = stick_module
        self.root_frame = root_frame
        self.stick_angle = stick_angle
        self.modules = [stick_module]

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
        from_face_index = 0  # always grow from face 0
        from_stick = "stick1"  # always grow from stick3
        face_frame_to_connect = self.get_face_frame(from_module_index, from_face_index, stick="stick3") # get the frame from base module
        face_frame_to_grow_from = self.get_face_frame(0, 0, from_stick) # get the frame from module index to grow from
        face_frame_to_connect.point -= face_frame_to_connect.xaxis * (25) 
        
        # calculate angle between normals of the two frames
        angle = abs(face_frame_to_connect.zaxis.angle(face_frame_to_grow_from.zaxis))
        dot = face_frame_to_connect.zaxis.dot(face_frame_to_grow_from.zaxis)
       
        angle = (math.pi - angle)
      
      
            
        # translation vector of the base_frame 
        v1 = Vector.from_start_end(self.root_frame.point, face_frame_to_connect.point)
        v2 = Vector.from_start_end(self.root_frame.point, face_frame_to_grow_from.point)
        translation_vector = v1 - v2
        new_frame = self.root_frame.translated(translation_vector)
        
        # rotate the frame to grow from to align with the connect frame
        R = Rotation.from_axis_and_angle(face_frame_to_connect.yaxis, angle=angle, point=face_frame_to_connect.point)
        new_frame.transform(R)
        
        # create new module at the new frame
        module = StickModuleA(new_frame, angle=self.stick_angle)
        new_module = module.create_module()
        modules = [module.sticks[stick] for stick in new_module]
        
        # append to the list of modules 
        self.modules.append(new_module)

        return modules
        
        
        
        
        
        
    
   
        
    # def get_faces_indexes(self, module_index, stick="stick1"):
        """
        Gets all face indexes of a chosen stick within a module.
        Args:
            module_index: Index of the stick
            stick: Stick object within the module
        Returns:
            List of face indexes as center points of the frames
        """        
        face_indexes = []
        for i in range(4):
            face_frame = self.get_face_frame(module_index, i, stick)
            face_indexes.append((face_frame.point))
        
        return face_indexes

        