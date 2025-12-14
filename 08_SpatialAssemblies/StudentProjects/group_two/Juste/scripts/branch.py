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

        self._init_first_stick(root_frame)

    def _init_first_stick(self, frame):
        """
        Creates the first stick from the root frame.
        """
        axis = Line.from_point_and_vector(frame.point, frame.xaxis * self.stick_length)
        z_vector = frame.yaxis
        self.sticks.append(Stick(axis, z_vector))

    def get_face_frame(self, stick_index, face_index):
        """
        Gets a frame located on a specified face of a stick.

        Args:
            stick_index: Index of stick to extract from
            face_index: Which face (0 = +Y, 1 = -Y, 2 = +Z, 3 = -Z)

        Returns:
            Frame on the specified face
        """
        if stick_index >= len(self.sticks):
            raise IndexError(f"Stick index {stick_index} out of range.")

        stick = self.sticks[stick_index]
        base_frame = stick.frame.copy()
        end_pt = stick.axis.end

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
        zaxis = xaxis.cross(yaxis).unitized()

        face_frame = Frame(end_pt + yaxis * (self.depth * 0.5), xaxis, yaxis)
        return face_frame

    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=0.0):
        """
        Grows a new stick from a given stick's face.

        Args:
            from_stick_index: Stick to grow from (-1 = last)
            face_index: Which face to grow from (0-3)
            angle: Optional rotation around Y axis (deg)
            offset: Optional lateral offset
        """
        if not self.sticks:
            # Should never hit this if _init_first_stick is called in __init__
            return

        from_stick_index = from_stick_index if from_stick_index != -1 else len(self.sticks) - 1
        position = self.get_face_frame(from_stick_index, face_index).copy()

        # Offset outward
        position.point += position.yaxis * (self.depth * 0.5)
        position.point -= position.xaxis * offset

        # Optional rotation
        if angle != 0.0:
            R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point=position.point)
            position.transform(R)

        axis = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        z_vector = position.yaxis
        self.sticks.append(Stick(axis, z_vector))

    def visualize(self):
        """
        Returns all stick box geometries.
        """
        return [stick.geometry for stick in self.sticks]
