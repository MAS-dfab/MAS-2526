from compas.geometry import Box, Rotation, Line
import math

class Stick:
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, frame, length = 200, width = None, depth = None):

        """
        Constructor for singular stick.

        Args:
        frame: starting frame of stick
        length: length dimension of stick
        width: width dimension
        depth: depth dimension of stick
        """
        self.frame = frame
        self.z_vector = frame.normal
        self.length = length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.axis = self._axis_from_frame()
        self.midframe = self._get_axis_mid_frame()
        self.corners = self._get_corners()
        self.aabb = self._get_aabb()


    def _axis_from_frame(self):
        st_pt = self.frame.point
        dir = self.frame.xaxis
        axis_ln = Line.from_point_direction_length(st_pt, dir, self.length)
        return axis_ln
    
    def _get_axis_mid_frame(self):
        v = self.frame.xaxis.unitized()
        v *= (self.length/2)
        frame = self.frame.translated(v)
        return frame
    
    def eval_frame(self, face_index = 0, t_value = 0.5):
        """
        face_index: Face index (0-3) around the stick
            t_value: The relative position along the line as a fraction of the length of the line. 0.0 corresponds to the start point and 1.0 corresponds to the end point. Numbers outside of this range are also valid and correspond to points beyond the start and end point.

        returns:
            frame on face_index along t_value
        """
        #rotate stick frame based on index
        frame = self.midframe
        angle = (face_index % 4) * (math.pi / 2)
        R = Rotation.from_axis_and_angle(frame.xaxis, angle, frame.point)
        new_frame = frame.transformed(R)

        #move along stick axis
        new_frame.point = self.axis.point_at(t_value)

        #offset frame onto stick face
        new_frame.point += new_frame.zaxis * (self.depth /2)
        return new_frame

    def _get_corners(self):
        l, w, d = self.length / 2, self.width / 2, self.depth / 2
        vec_x, vec_y, vec_z = self.frame.xaxis, self.frame.yaxis, self.frame.zaxis
        sign = [-1, 1]
        corners = []

        for i in sign:
            for j in sign:
                for k in sign:
                    corner = self.midframe.point + vec_x * (i * l) + vec_y * (j * w) + vec_z (l * d)
                    corners.append(corner)

        return corners
    
    def _get_aabb(self):
        corners = self.corners

        x_s = [p[0] for p in corners]
        y_s = [p[1] for p in corners]
        z_s = [p[2] for p in corners]

        return [(min(x_s), min(y_s), min(z_s), max(x_s), max(y_s), max(z_s))]
    
    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box
    
    def rotate_stick(self, angle, rotation_axis=None, pt=None):
        if not rotation_axis:
            rotation_axis = self.axis.direction
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), pt or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)