import math
from compas.geometry import Line, Rotation
from Sticks2 import Stick   # make sure this matches your actual module name


class BranchingModule:
    """
    Branching module that grows new sticks from the faces of an existing root stick.
    Branching position along the parent stick is controlled by a 0–1 offset parameter.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset=1.0):
        """
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
        offset : float, optional
            Default normalized position [0–1] along the parent stick axis
            where new branches will start. 0 = base, 1 = tip.
        """
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset = offset  # default branch position along parent axis


    # -------------------------------------------------------------------------
    # Face Frame Extraction with normalized axis offset
    # -------------------------------------------------------------------------
    def get_face_frame(self, stick_index, face_index, offset=1.0):
        """
        Returns a Frame on one of the four faces of the given stick, at a
        normalized position along its axis.

        Parameters
        ----------
        stick_index : int
            Index of the stick in self.sticks.
        face_index : int {0,1,2,3}
            Which face around the stick to use.
        offset : float
            Normalized parameter [0–1] along the parent stick axis:
            0 = axis.start, 1 = axis.end.
        """

        stick = self.sticks[stick_index]
        base_frame = stick.frame

        # Rotate around stick's local x-axis to choose face
        angle = face_index * (math.pi / 2.0)  # 0, 90, 180, 270 deg
        R_face = Rotation.from_axis_and_angle(
            base_frame.xaxis,
            angle,
            point=base_frame.point
        )
        face_frame = base_frame.transformed(R_face)

        # Clamp offset to [0, 1] just to be safe
        t = max(0.0, min(1.0, offset))

        # Point along parent axis
        point_on_axis = stick.axis.point_at(t)

        # Move frame origin to that point
        face_frame.point = point_on_axis

        # Offset outward along local Y to sit on the face surface
        face_frame.point += face_frame.yaxis * (self.depth * 0.5)

        return face_frame


    # -------------------------------------------------------------------------
    # Grow New Stick
    # -------------------------------------------------------------------------
    def grow_stick(self, from_stick_index=-1, face_index=0, angle=0.0, offset=None):
        """
        Grows a new stick from a selected face of an existing stick.

        Parameters
        ----------
        from_stick_index : int
            Index of the existing stick to branch from.
        face_index : int {0,1,2,3}
            Face index around the stick.
        angle : float
            Additional rotation in degrees around the face's local Y-axis.
        offset : float, optional
            Normalized position [0–1] along the parent stick axis
            (overrides the default self.offset if provided).
        """

        # Use instance default if no offset passed
        if offset is None:
            offset = self.offset

        # Get face frame at the chosen axis position
        base_frame = self.get_face_frame(from_stick_index, face_index, offset=offset)
        base_frame = base_frame.copy()

        # Extra rotation around the face's Y-axis
        if angle != 0.0:
            R = Rotation.from_axis_and_angle(
                base_frame.yaxis,
                math.radians(angle),
                point=base_frame.point
            )
            base_frame.transform(R)

        # New stick axis aligned with face frame's X-axis
        new_axis = Line.from_point_and_vector(
            base_frame.point,
            base_frame.xaxis * self.stick_length
        )

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
        """Return geometry of all sticks as COMPAS Box objects."""
        return [s.geometry for s in self.sticks]
