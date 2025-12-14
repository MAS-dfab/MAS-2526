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

        # IMPORTANT:
        # Do NOT create geometry here.
        # Geometry creation is deferred to grow_stick().


    # -------------------------------------------------
    # INTERNAL: Create first stick ONLY when needed
    # -------------------------------------------------
    def _init_first_stick(self):
        """
        Creates the first stick aligned to the root frame.
        """
        axis = Line.from_point_and_vector(
            self.root_frame.point,
            self.root_frame.xaxis * self.stick_length
        )
        z_vector = self.root_frame.yaxis
        self.sticks.append(Stick(axis, z_vector))


    # -------------------------------------------------
    # FACE FRAME (no rotation creep)
    # -------------------------------------------------
    def get_face_frame(self, stick_index, face_index):
        """
        Gets a frame located on a specified face of a stick.

        face_index:
            0 = +Y
            1 = -Y
            2 = +Z
            3 = -Z
        """
        stick = self.sticks[stick_index]
        base_frame = stick.frame
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

        return Frame(
            end_pt + yaxis * (self.depth * 0.5),
            xaxis,
            yaxis
        )


    # -------------------------------------------------
    # MAIN GROWTH LOGIC (Option A)
    # -------------------------------------------------
    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=0.0):
        """
        Grows a new stick from a specified face.

        If no sticks exist yet:
            → create the first stick
            → STOP (do not grow a second stick accidentally)
        """

        # FIRST STICK (SAFE ENTRY POINT)
        if not self.sticks:
            self._init_first_stick()
            return

        # Determine which stick to grow from
        from_stick_index = (
            from_stick_index
            if from_stick_index != -1
            else len(self.sticks) - 1
        )

        position = self.get_face_frame(from_stick_index, face_index).copy()

        # Maintain face‑to‑face clearance
        position.point += position.yaxis * (self.depth * 0.5)
        position.point -= position.xaxis * offset

        # Optional in‑plane rotation (does NOT affect axis orientation)
        if angle != 0.0:
            R = Rotation.from_axis_and_angle(
                position.yaxis,
                math.radians(angle),
                point=position.point
            )
            position.transform(R)

        # Create child stick
        axis = Line.from_point_and_vector(
            position.point,
            position.xaxis * self.stick_length
        )

        z_vector = position.yaxis
        self.sticks.append(Stick(axis, z_vector))


    # -------------------------------------------------
    # VISUALIZATION
    # -------------------------------------------------
    def visualize(self):
        """
        Returns COMPAS Box geometries (GH‑previewable).
        """
        return [stick.geometry for stick in self.sticks]
