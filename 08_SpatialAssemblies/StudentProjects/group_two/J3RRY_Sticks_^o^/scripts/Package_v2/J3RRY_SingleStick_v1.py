from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import angle_vectors, Rotation
import math


class Stick:
    size = 13.0
    width = size
    depth = size
    def __init__(self, frame, length=None, width=None, depth=None):
        """
        Constructor for single Stick.
        Args:
            frame: Frame representing the start of the stick.
            length: Length of the stick (defaults to 50.0)
            width: Width of the stick (defaults to Stick.width)
            depth: Depth of the stick (defaults to Stick.depth)
        """
        self.frame = frame
        self.length = length or 50.0
        self.width = width or Stick.width
        self.depth = depth or Stick.depth

        self.axis = self._axis_from_frame()
        self.midframe = self._get_axis_mid_frame()
        self.corners = self._get_corners()
        self.aabb = self._get_aabb()


    def _axis_from_frame(self):
        """
        Private method to compute the stick's axis as a Line.

        Returns:
            Line representing the stick's axis.
        """
        start_pt = self.frame.point
        direction = self.frame.xaxis
        axis = Line.from_point_direction_length(start_pt, direction, self.length)
        return axis
    
    def _get_axis_mid_frame(self):
        """
        Private method to compute the mid frame of the stick's axis.

        Returns:
            Frame at the middle point of the stick's axis, aligned with the stick's frame but at different origin.
        """
        vector = self.frame.xaxis.unitized()
        vector *= (self.length / 2)
        frame = self.frame.translated(vector)
        return frame
    
    def eval_frame(self, face_index=0, t_value=.5):
        """
        Gets a frame on one of the four faces of a stick.

        Args:
            face_index: Face index (0-3) around the stick
            t_value: The relative position along the line as a fraction of the length of the line. 0.0 corresponds to the start point and 1.0 corresponds to the end point. Numbers outside of this range are also valid and correspond to points beyond the start and end point.
            
        Returns:
            Frame on the specified face index and t value.
        """
        # Rotate stick frame based on index
        base = self.midframe
        angle = (face_index % 4) * (math.pi / 2)
        R = Rotation.from_axis_and_angle(base.xaxis, angle, base.point)
        new_stick_frame = base.transformed(R)
        new_stick_frame.point = self.axis.point_at(t_value)
        # Offset frame to be on surface of stick
        new_stick_frame.point += new_stick_frame.zaxis * (self.depth / 2)
        return new_stick_frame
    
    def _get_corners(self):
        """
        Private method to compute 8 corners of the stick.
        
        Returns:
            List of 8 corner points of the stick.
        """
        l, w, d = self.length / 2, self.width / 2, self.depth / 2
        vecx, vecy, vecz = self.frame.xaxis, self.frame.yaxis, self.frame.zaxis
        sign = [-1, 1]
        corners = []
        for i in sign:
            for j in sign:
                for k in sign:
                    corner = self.midframe.point + vecx * (i*l) + vecy * (j*w) + vecz * (k*d)
                    corners.append(corner)
        return corners
    
    def _get_aabb(self):
        """
        Private method to compute the axis-aligned bounding box of the stick.

        Returns:
            (min_point, max_point): tuple of two points defining the AABB
        """
        corners = self.corners
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        zs = [p[2] for p in corners]
        return [(min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))]
    


    @property
    def geometry(self):
        return Box(self.length, self.width, self.depth, self.midframe)