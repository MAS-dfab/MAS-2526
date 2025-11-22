from compas.geometry import Line, Frame, Vector, Point, Rotation, Plane, Translation, mirror_point_plane
import math
from stick import Stick
import copy


class OStickModule:
    def __init__(self, pt, stick_length, stick_width, stick_depth, offset, type={"x": 0, "y": 0, "z": 0}):
        """
        Orthogonal stick module using rectangular sticks.

        Args:
            pt: center point for the module
            stick_length: length of each stick
            stick_width: width of stick cross-section
            stick_depth: depth of stick cross-section
            offset: positional offset applied to sticks
            type: dict controlling presence/flags per axis (x, y, z: 0/1/2)
        """
        self.pt = pt
        self.stick_length = stick_length
        self.stick_width = stick_width
        self.stick_depth = stick_depth
        self.stick_offset = offset
        self.type = type
        self.stick_module = self.create_orthogonal_module(
            pt, stick_length, stick_width, stick_depth, offset
        )

    def create_orthogonal_module(self, pt, stick_length, stick_width, stick_depth, offset):
        w = stick_width
        d = stick_depth


        offsetpt_x = (
            pt 
            - Vector(offset, 0, 0) 
            + Vector(d/2, 0, 0) 
            + Vector(0, 0, d/2)
            + Vector(0, 2 * d * self.type["x"], 0)
        )
        offsetpt_y = (
            pt
            - Vector(0, offset, 0)
            - Vector(0, 0,  d/2)
            + Vector(2 * d * self.type["y"], 0, 0)
        )
        offsetpt_z = (
            pt
            - Vector(0, 0, offset)
            + Vector(0,  d, 0)
            + Vector( d, 0, 0)
            - Vector(0, 2 * d * self.type["z"], 0)
        )

        stick_x = Stick(
            Line(offsetpt_x, offsetpt_x + Vector(stick_length, 0, 0)), width=stick_width, depth=stick_depth
        )
        if self.type["x"] != 2:
            yield stick_x
        stick_y = Stick(
            Line(offsetpt_y, offsetpt_y + Vector(0, stick_length, 0)), width=stick_width, depth=stick_depth
        )
        if self.type["y"] != 2:
            yield stick_y
        stick_z = Stick(
            Line(offsetpt_z, offsetpt_z + Vector(0, 0, stick_length)), width=stick_width, depth=stick_depth
        )
        if self.type["z"] != 2:
            yield stick_z


class RStickModule:
    def __init__(self, number_of_sticks, stick_length, sticks=[]):
        self.number_of_sticks = number_of_sticks
        self.stick_length = stick_length
        self.sticks = sticks

    @classmethod
    def from_stick_module(cls, sticks):
        return cls(len(sticks), sticks[0].axis.length, sticks)

    @property
    def geometry(self):
        return [stick.geometry for stick in self.sticks]

    @property
    def center(self):
        x = sum([s.axis.end.x for s in self.sticks])/len(self.sticks)
        y = sum([s.axis.end.y for s in self.sticks])/len(self.sticks)
        z = (self.sticks[0].axis.end.z + self.sticks[0].axis.start.z)/2.0
        return Point(x,y,z)

    def flip(self):
        for stick in self.sticks:
            start_z = stick.axis.start.z
            stick.axis.start.z = stick.axis.end.z
            stick.axis.end.z = start_z

    def rotate(self, angle):
        R = Rotation.from_axis_and_angle(Vector(0, 0, 1), angle, self.center)
        for stick in self.sticks:
            stick.axis.transform(R)

    def move(self, vector):
        T = Translation.from_vector(vector)
        for stick in self.sticks:
            stick.axis.transform(T)

    def copy(self):
        return RStickModule.from_stick_module(copy.deepcopy(self.sticks))

    def mirror_center_along_stick(self, stick):
        plane = Plane(stick.axis.midpoint, stick.axis.direction.cross(Vector(0,1,0)))
        return Point(*mirror_point_plane(self.center, plane))

    def generate_stick_frames(self):
        for stick in self.sticks:
            stick.frame = Frame(stick.axis.midpoint, -stick.axis.direction, stick.axis.direction.cross(self.center - stick.axis.midpoint))
            stick.frame.transform(Rotation.from_axis_and_angle(stick.axis.direction, -math.pi/4, stick.axis.midpoint))
