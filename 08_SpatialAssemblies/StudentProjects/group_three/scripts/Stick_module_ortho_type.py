from compas.geometry import Line, Vector, Rotation
import math
from Sticks import Stick

class BranchingModule_ortho_type:
    def __init__(self, root_frame, stick_length=None, width=None, depth=None):

        self.root_frame = root_frame
        self.sticks = []
        
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        
        self._init_first_stick(root_frame)

    def _init_first_stick(self, frame):
        
        stick_axis = Line.from_point_and_vector(frame.point, frame.zaxis * self.stick_length)
        
        my_stick = Stick(stick_axis, frame.yaxis, self.width, self.depth)
        self.sticks.append(my_stick)

    def get_face_frame(self, stick_index, face_index, position_ratio = 1):
        stick_frame = self.sticks[stick_index].frame.copy()
        angle = face_index
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, math.radians(angle), stick_frame.point)
        new_frame = stick_frame.transform(R)

        start = self.sticks[stick_index].axis.start
        end = self.sticks[stick_index].axis.end
        new_frame.point += start + (end - start) * position_ratio

        new_frame.point += new_frame.yaxis * (self.depth / 2.0)

        return new_frame

    def grow_stick(
        self,
        from_stick_index=-1,
        face_index=0,
        angle=0.0,
        position_0=1.0,
        position_1=0.0,
    ):

        # 1. get a frame at the chosen face and at a chosen position along the stick
        position = self.get_face_frame(from_stick_index, face_index, position_0).copy()

        position.point += position.yaxis * ( - self.depth / 2.0 + (self.depth * position_1) )
        
        # 2. rotate the frame around its x axis by the given angle
        rad = math.radians(angle)
        R = Rotation.from_axis_and_angle(position.xaxis, rad, position.point)
        position.transform(R)

        connection_point = position.point.copy()
        offset_distance = position_1 * self.stick_length
        start_point = connection_point - position.yaxis * offset_distance

        # 3. create a new stick at the new position
        stick_axis = Line.from_point_and_vector(start_point, position.xaxis * self.stick_length)
        zvector = position.yaxis

        new_stick = Stick(stick_axis, zvector, self.width, self.depth)
        self.sticks.append(new_stick)   

        # return new_stick
    
    def visualize(self):
        """
        Returns all stick geometries.       
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]