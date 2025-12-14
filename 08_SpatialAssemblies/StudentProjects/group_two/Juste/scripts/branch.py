from compas.geometry import Line, Frame, Vector, Rotation
import math
from Sticks import Stick

class BranchingModule:
    def __init__(self, root_frame, stick_length=None, width=None, depth=None, disable_init=False):
        self.root_frame = root_frame
        self.sticks = []
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        if not disable_init:
            self._init_first_stick(root_frame)

    def _init_first_stick(self, frame):
        axis = Line.from_point_and_vector(frame.point, frame.xaxis * self.stick_length)
        st_stick = Stick(axis, z_vector=frame.yaxis)
        self.sticks.append(st_stick)

    def get_face_frame(self, stick_index, face_index):
        """
        Gets the frame at a face of a stick.
        
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

        xaxis = base_frame.xaxis
        yaxis = normal
        face_origin = end_pt + yaxis * (self.depth / 2)
        return Frame(face_origin, xaxis, yaxis)

    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=0.0):
        """
        Grows a new stick from an existing stick, or from the root frame if first stick.
        """
        if not self.sticks:
            # First stick: grow from root_frame
            axis = Line.from_point_and_vector(
                self.root_frame.point, self.root_frame.xaxis * self.stick_length
            )
            z_vector = self.root_frame.yaxis
            self.sticks.append(Stick(axis, z_vector))
            return

        # Subsequent sticks
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * (self.depth / 2)
        position.point += -position.xaxis * offset

        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), position.point)
        position.transform(R)

        position.point += -position.xaxis * offset

        axis = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        z_vector = position.yaxis
        self.sticks.append(Stick(axis, z_vector))

    def visualize(self):
        """
        Returns all stick geometries.
        """
        return [stick.geometry for stick in self.sticks]
