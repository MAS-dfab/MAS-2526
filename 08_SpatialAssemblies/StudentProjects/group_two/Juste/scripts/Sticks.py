from compas.geometry import Point, Box, Frame, Vector, Line, Rotation
from compas.geometry import angle_vectors
from compas_rhino.conversions import box_to_rhino
import math

def _calculate_z_vector_from_centerline(centerline_vector):
    # fallback normal if axis is almost vertical
    c = Vector(0, 0, 1)
    angle = angle_vectors(c, centerline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        c = Vector(1, 0, 0)
    return c

class Stick:
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, axis: Line, z_vector: Vector = None, width: float = None, depth: float = None):
        """
        axis: compas Line representing the centerline of the stick
        z_vector: optional vector to define stick normal
        """
        self.axis = axis
        self.z_vector = z_vector or _calculate_z_vector_from_centerline(axis.direction)
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        # Compute canonical frames
        self.base_frame = self._frame_at_point(axis.start)
        self.mid_frame = self._frame_at_point(axis.midpoint)
        self.tip_frame = self._frame_at_point(axis.end)

        # central frame used for visualization
        self.frame = self.mid_frame

    def _frame_at_point(self, pt: Point) -> Frame:
        # create a frame with x axis along axis.direction and z as normal
        return Frame(pt, self.axis.direction, self.z_vector)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box_to_rhino(box)

    def face_point(self, face_index: int):
        """
        Return the point on the stick corresponding to a face index:
        0: +Y face
        1: -Y face
        2: +Z face
        3: -Z face
        """
        # local frame axes
        f = self.mid_frame
        axes = {
            0: f.yaxis,
            1: -f.yaxis,
            2: f.zaxis,
            3: -f.zaxis
        }
        if face_index not in axes:
            raise ValueError("face_index must be 0,1,2,3")
        return f.point + axes[face_index] * (self.depth * 0.5)

    def rotate_stick(self, angle_degrees: float, axis=None, about=None):
        """
        Rotate the entire stick around a given axis (default: its own axis).
        about = Point around which to rotate; if None, uses midpoint.
        """
        axis = axis or self.axis.direction
        origin = about or self.axis.midpoint
        R = Rotation.from_axis_and_angle(axis, math.radians(angle_degrees), origin)
        self.axis.transform(R)
        self.base_frame.transform(R)
        self.mid_frame.transform(R)
        self.tip_frame.transform(R)
        self.frame.transform(R)
