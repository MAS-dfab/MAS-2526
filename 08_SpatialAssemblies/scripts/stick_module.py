import math
from compas.geometry import Line, Rotation
from Sticks import Stick


class BranchingModule:
    """
    Branching module that grows new sticks from the faces of an existing root stick.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None):
        """
        Initialize a branching module starting from an existing Stick object.

        Parameters
        ----------
        root_stick : Stick
            The first stick of the structure. Branching begins from its faces.
        stick_length : float, optional
            Length of each stick grown in branching.
        width : float, optional
            Width of each stick (defaults to Stick.WIDTH).
        depth : float, optional
            Depth of each stick (defaults to Stick.DEPTH).
        """
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH


    # -------------------------------------------------------------------------
    # Face Frame Extraction
    # -------------------------------------------------------------------------
    def get_face_frame(self, stick_index, face_index):
        """
        Returns a Frame on one of the four faces of the given stick.
        
        Face numbering (around stick frame.xaxis):
            0 = original +Y face
            1 = rotated 90 degrees
            2 = rotated 180 degrees
            3 = rotated 270 degrees
        """

        stick = self.sticks[stick_index]
        frame = stick.frame

        # Rotate the frame around the stick's local x-axis
        angle = face_index * (math.pi / 2)     # 0, 90, 180, 270 degrees
        R = Rotation.from_axis_and_angle(frame.xaxis, angle, point=frame.point)
        face_frame = frame.transformed(R)

        # Place the frame at the tip of the stick
        face_frame.point = stick.axis.end

        # Offset outward along local Y to land on face surface
        face_frame.point += face_frame.yaxis * (self.depth * 0.5)

        return face_frame


    # -------------------------------------------------------------------------
    # Grow New Stick
    # -------------------------------------------------------------------------
    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0):
        """
        Grows a new stick from a selected face of an existing stick.

        Parameters
        ----------
        from_stick_index : int
            Index of the existing stick to branch from.
        face_index : int {0,1,2,3}
            Face index around the stick.
        angle : float
            Additional rotation in degrees around the local Y-axis (open/close branch).
        """

        # Get face frame at tip of chosen stick
        base_frame = self.get_face_frame(from_stick_index, face_index)
        base_frame = base_frame.copy()

        # Additional user-controlled rotation about the face's Y-axis
        if angle != 0.0:
            R = Rotation.from_axis_and_angle(base_frame.yaxis,
                                             math.radians(angle),
                                             point=base_frame.point)
            base_frame.transform(R)

        # Generate new stick axis aligned with the local X-axis of the face frame
        new_axis = Line.from_point_and_vector(base_frame.point,
                                              base_frame.xaxis * self.stick_length)

        # Create stick
        new_stick = Stick(
            new_axis,
            width=self.width,
            depth=self.depth
        )

        self.sticks.append(new_stick)


    # -------------------------------------------------------------------------
    # Visualization Helper
    # -------------------------------------------------------------------------
    def visualize(self):
        """
        Returns geometry of all sticks as COMPAS Box objects.
        """
        return [s.geometry for s in self.sticks]
