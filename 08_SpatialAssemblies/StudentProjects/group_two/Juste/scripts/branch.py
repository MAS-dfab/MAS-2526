from compas.geometry import Line, Frame, Vector
from compas.geometry import Rotation
import math

from Sticks import Stick


class BranchingModule:
    def __init__(self, root_frame, stick_length=None, width=None, depth=None):
        """
        Branching module that grows sticks from a root frame.

        The first (root) stick is always created automatically.
        Subsequent sticks grow from faces of existing sticks.
        """
        self.root_frame = root_frame
        self.sticks = []

        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        # ALWAYS create exactly one root stick
        self._init_first_stick()

    # --------------------------------------------------
    # ROOT STICK
    # --------------------------------------------------

    def _init_first_stick(self):
        """
        Creates the initial root stick from self.root_frame.
        """
        frame = self.root_frame

        axis = Line.from_point_and_vector(
            frame.point,
            frame.xaxis * self.stick_length
        )

        z_vector = frame.yaxis
        self.sticks.append(Stick(axis, z_vector))

    # --------------------------------------------------
    # FACE FRAMES
    # --------------------------------------------------

    def get_face_frame(self, stick_index, face_index):
        """
        Returns a frame on a specific face of a stick.

        Face index convention:
            0 → +Y
            1 → -Y
            2 → +Z
            3 → -Z
        """
        if stick_index >= len(self.sticks):
            raise IndexError("Invalid stick index")

        stick = self.sticks[stick_index]
        base = stick.frame.copy()
        end_pt = stick.axis.end

        if face_index == 0:
            normal = base.yaxis
        elif face_index == 1:
            normal = -base.yaxis
        elif face_index == 2:
            normal = base.zaxis
        elif face_index == 3:
            normal = -base.zaxis
        else:
            raise ValueError("face_index must be 0, 1, 2, or 3")

        xaxis = base.xaxis
        yaxis = normal
        zaxis = xaxis.cross(yaxis).unitized()

        return Frame(
            end_pt + yaxis * (self.depth * 0.5),
            xaxis,
            yaxis
        )

    # --------------------------------------------------
    # GROWTH
    # --------------------------------------------------

    def grow_stick(self, from_stick_index=0, face_index=0, angle=0.0, offset=0.0):
        """
        Grows a new stick from a face of an existing stick.

        Args:
            from_stick_index: index of parent stick (-1 = last stick)
            face_index: which face to grow from (0–3)
            angle: optional in-plane rotation (degrees)
            offset: lateral offset along X
        """
        if not self.sticks:
            return  # safety, should never happen

        parent_index = (
            from_stick_index
            if from_stick_index != 0
            else len(self.sticks) - 1
        )

        face_frame = self.get_face_frame(parent_index, face_index).copy()

        # Ensure clean face-to-face separation
        face_frame.point += face_frame.yaxis * (self.depth * 0.5)
        face_frame.point -= face_frame.xaxis * offset

        if angle != 0.0:
            R = Rotation.from_axis_and_angle(
                face_frame.yaxis,
                math.radians(angle),
                point=face_frame.point
            )
            face_frame.transform(R)

        axis = Line.from_point_and_vector(
            face_frame.point,
            face_frame.xaxis * self.stick_length
        )

        z_vector = face_frame.yaxis
        self.sticks.append(Stick(axis, z_vector))

    # --------------------------------------------------
    # VISUALIZATION
    # --------------------------------------------------

    def visualize(self):
        """
        Returns COMPAS Box geometries (GH previews these directly).
        """
        return [stick.geometry for stick in self.sticks]
