from compas.geometry import Line, Rotation, Translation
from Sticks import Stick
import math



# THL_branchedModule_x_type
class BranchingModule_x_type:
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

    def get_face_frame(self, stick_index, face_index, distance_from_end=0.0):
        
        stick = self.sticks[stick_index]
        stick_frame = stick.frame

        # Rotate around the stick x-axis to get 4 faces
        angle = face_index * math.pi / 2.0
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle=angle, point=stick_frame.point)
        new_frame = stick_frame.transformed(R)

        # new_frame = stick_frame.copy()
        # new_frame.transform(R)
        
        # Choose a point along the stick axis:
        axis = stick.axis
        v_dir = axis.direction.unitized()

        if distance_from_end > 0.0:
            # move from end towards the center along -direction
            base_point = axis.end - v_dir * distance_from_end
        else:
            # old behavior: use the end point
            base_point = axis.end

        new_frame.point = base_point

        # move outwards to reach the surface of the stick
        new_frame.point += new_frame.yaxis * (self.depth / 2.0)

        return new_frame

    def grow_stick(
        self,
        from_stick_index=-1,
        face_index=0,
        angle=0.0,
        distance_from_end=0.0,
        align_center_with=None,
        offset_normal=0.0
    ):

        if from_stick_index == -1:
            from_stick_index = len(self.sticks) - 1

        # 1. get a frame at the chosen face and at a chosen position along the stick
        position = self.get_face_frame(from_stick_index, face_index, distance_from_end).copy()

        # 2. move further outwards so we are fully outside the parent stick
        position.point += position.yaxis * (self.depth / 2.0)
        # small offset along previous stick direction (old behavior)
        position.point += position.xaxis * -10.0

        # 3. rotate around the face normal (y-axis of the frame at the face)
        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), position.point)
        position.transform(R)

        # 4. small offset along new stick direction
        position.point += position.xaxis * -10.0

        # 5. construct centerline
        dir_vec = position.xaxis  # Frame axes are unit vectors in compas

        if align_center_with is not None:
            # We want the midpoint of this new stick to match the midpoint
            # of another stick (e.g. stick 0, the root stick).
            target_center = self.sticks[align_center_with].axis.midpoint
            half_len_vec = dir_vec * (self.stick_length / 2)

            start_point = target_center - half_len_vec
            centerline = Line.from_point_and_vector(start_point, dir_vec * self.stick_length)

        else:
            # Default behavior: start from the current frame point
            centerline = Line.from_point_and_vector(position.point, dir_vec * self.stick_length)

        # 6. create stick and store
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector, self.width, self.depth)


        if offset_normal != 0.0:            # Move stick along its normal (y axis)
            T = Translation.from_vector(new_stick.frame.yaxis * offset_normal)
            new_stick.axis.transform(T)
            new_stick.frame.transform(T)

        self.sticks.append(new_stick)
        return len(self.sticks) - 1  # return index of new stick

    def visualize(self):
        """Return all stick geometries as Boxes."""
        return [stick.geometry for stick in self.sticks]





