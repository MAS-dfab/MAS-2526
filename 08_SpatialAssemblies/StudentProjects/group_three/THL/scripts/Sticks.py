from compas.geometry import Point, Box, Frame, Vector, Plane, Rotation, Line, Scale
from compas.geometry import angle_vectors, intersection_line_plane, closest_point_on_segment
import math

def _calculate_z_vector_from_centerline(centerline_vector):
    c = Vector(0,0,1)
    angle = angle_vectors(c, centerline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        c = Vector(1,0,0)
    return c


class Stick:
    SIZE = 13.0

    WIDTH = SIZE
    DEPTH = SIZE

    RADIUS = 0.5 * math.sqrt(WIDTH**2 + DEPTH**2)

    def __init__(self, axis, z_vector = None, width = None, depth = None):
        # axis : Line(start, end), axis.length 棍子的長度, axis.direction 棍子的X方向
        self.axis = axis   # Line: contral stick's length and direction
        self.z_vector = z_vector  # Vector: to define the stick's frame orientation( y or z )
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self._get_stick_frame()

        
    def _get_stick_frame(self):
        if self.z_vector:
            normal = self.z_vector
        else:
            normal = _calculate_z_vector_from_centerline(self.axis.direction)
        
        # frame = (poin 在此為中心點, xaxis 在此為棍子的x方向, zaxis = normal)
        frame = Frame(self.axis.midpoint, self.axis.direction, normal)
        return frame


    @property
    # return the geometry of the stick as a box ; box(length, width, depth, frame) ; length is along x axis ; width is along y axis ; depth is along z axis ;  frame is the position and orientation of the box
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box
    
    def rotate_stick(self, angle, rotation_axis=None, pt=None):
        if not rotation_axis:
            rotation_axis = self.axis.direction
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), pt or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)



# for reciporcal eccentricity calculation
    def shift_stick(self, length):
        direction = self.axis.direction
        start = self.axis.start + direction * length
        end = self.axis.end + direction * length
        self.axis = Line(start, end)
        self.frame.point = self.frame.point + direction * length

    def set_stick_length(self, length):
        direction = self.axis.direction
        end = self.axis.end
        start = end - direction * length
        self.axis = Line(start, end)
        self.frame.point = self.axis.midpoint

    def scale_stick(self, factor):
        s = Scale.from_factors([factor, factor, factor])
        self.axis = self.axis.transformed(s)
        self.frame = self.frame.transformed(s)
        self.width *= factor
        self.depth *= factor

    def eccentricity(self, other_stick):
        cross_p = self.axis.direction.cross(other_stick.axis.direction)

        q1 = intersection_line_plane(
            self.axis,
            Plane(
                point=other_stick.axis.midpoint,
                normal=cross_p.cross(other_stick.axis.direction),
            ),
        )

        q2 = intersection_line_plane(
            other_stick.axis,
            Plane(
                point=self.axis.midpoint,
                normal=cross_p.cross(self.axis.direction),
            ),
        )

        return Line(q1, q2)

    def eccentrictiry_rotation_angle(self, alpha):
        eff_r = Stick.RADIUS

        half_len = self.axis.length / 2.0
        denom = math.sqrt(half_len ** 2 - (eff_r ** 2) / (math.sin(alpha) ** 2))

        if denom == 0:
            return 0.0

        teta = eff_r / denom
        return math.atan(teta)






#  frame to plane ( frame.point as origin, frame.normal as normal 法向)
def stick_bridge(stick0, stick1):
    plane0 = Plane.from_frame(stick0.frame)
    plane1 = Plane.from_frame(stick1.frame)
    # 用stick0.axis跟stick1的平面plane1求交點p0
    p0 = intersection_line_plane(stick0.axis, plane1)
    # 用stick1.axis跟stick0的平面plane0求交點p1
    p1 = intersection_line_plane(stick1.axis, plane0)

    return Stick(Line(p0, p1))



def stick_bridge_closest(stick0, stick1):
    seg0 = (stick0.axis.start, stick0.axis.end)
    seg1 = (stick1.axis.start, stick1.axis.end)
    
    p0 = closest_point_on_segment(stick1.axis.midpoint, seg0)
    p1 = closest_point_on_segment(stick0.axis.midpoint, seg1)

    v = Vector.from_start_end(p0, p1)
    if v.length < 0.001:
        mid0 = stick0.axis.midpoint
        mid1 = stick1.axis.midpoint
        return Stick(Line(mid0, mid1))
    else:
        return Stick(Line(p0, p1))

def stick_bridge_midpoint(stick0, stick1):
    mid0 = stick0.axis.midpoint
    mid1 = stick1.axis.midpoint
    return Stick(Line(mid0, mid1))