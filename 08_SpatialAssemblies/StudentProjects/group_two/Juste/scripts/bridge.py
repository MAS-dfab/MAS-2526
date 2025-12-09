from compas.geometry import Line, Point, Vector, Plane, Rotation, closest_point_on_line
import math

from Sticks import Stick

def compare_angles(frame_0, frame_1):
    return math.degrees(*Vector.angle_vectors([frame_0.normal], [frame_1.normal]))

def get_plane_from_frame(frame):
    plane = Plane.from_frame(frame)
    plane.normal = frame.yaxis
    return plane

class GrowTowards:
    def __init__(self, root_frame, target_frame, offset_root_child=0.0, offset_target_child=0.0, stick_length=None, width=None, depth=None):
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.sticks = []

        self.root_frame = root_frame
        self.root_frame_axis = root_frame.xaxis * self.stick_length
        self.target_frame = target_frame
        self.target_frame_axis = target_frame.xaxis * self.stick_length

        self.offset_root_child = offset_root_child
        self.offset_target_child = offset_target_child

        self.normal_deviation = self.compare_angles(self.root_frame, self.target_frame)
        self.rotated_target_frame = self.rotate_target_frame(self.target_frame)

        self.root_child_frame = self.get_root_child_frame(self.root_frame, self.root_frame_axis)
        self.target_child_frame = self.get_target_child_frame(self.rotated_target_frame, self.target_frame_axis)

        self.frame_intersection = self.get_frame_intersection(self.root_child_frame, self.target_child_frame)

        if self.frame_intersection:
            self.intersection_closest_point = self.get_intersection_closest_point(self.root_child_frame, self.frame_intersection)
            self.root_child_stick = self.get_root_child_stick(self.root_child_frame, self.frame_intersection)
            self.target_child_stick = self.get_target_child_stick(self.target_child_frame, self.frame_intersection)
        else:
            self.intersection_closest_point = None
            self.root_child_stick = None
            self.target_child_stick = None

    def compare_angles(self, frame_0, frame_1):
        return math.degrees(*Vector.angle_vectors([frame_0.normal], [frame_1.normal]))

    def rotate_target_frame(self, target_frame):
        if self.normal_deviation > 180:
            R = Rotation.from_axis_and_angle(target_frame.axis, -math.pi, target_frame.point)
        elif self.normal_deviation > 90:
            R = Rotation.from_axis_and_angle(target_frame.xaxis, -math.pi/2, target_frame.point)
        else:
            return target_frame.copy()
        return target_frame.transformed(R)

    def get_root_child_frame(self, frame, axis, flip=False):
        child_frame = frame.copy()
        child_frame.point = Line.from_point_and_vector(child_frame.point, axis).midpoint
        angle = (3 if flip else 1) * math.pi/2
        R = Rotation.from_axis_and_angle(child_frame.xaxis, angle, child_frame.point)
        new_frame = child_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth/2
        return new_frame

    def get_target_child_frame(self, frame, axis, flip=False):
        child_frame = frame.copy()
        child_frame.point = Line.from_point_and_vector(child_frame.point, axis).midpoint
        angle = (3 if flip else 1) * math.pi/2
        R = Rotation.from_axis_and_angle(child_frame.xaxis, angle, child_frame.point)
        new_frame = child_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth/2
        return new_frame

    def get_frame_intersection(self, frame_0, frame_1):
        plane_0 = Plane.from_frame(frame_0)
        plane_0.normal = frame_0.yaxis
        plane_1 = Plane.from_frame(frame_1)
        plane_1.normal = frame_1.yaxis
        return plane_0.intersection_with_plane(plane_1)

    def get_intersection_closest_point(self, frame, intersection):
        return Point(*closest_point_on_line(frame.point, intersection))

    def get_root_child_stick(self, frame, intersection):
        root_child_frame = frame.copy()
        vector_end = Point(*closest_point_on_line(root_child_frame.point, intersection))
        axis_vector = Vector.from_start_end(root_child_frame.point, vector_end).unitized()
        root_child_frame.point += root_child_frame.yaxis * self.depth/2
        root_child_frame.point += -axis_vector * self.offset_root_child
        axis = Line.from_point_and_vector(root_child_frame.point, axis_vector * self.stick_length)
        z_vector = root_child_frame.yaxis
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)
        return new_stick

    def get_target_child_stick(self, frame, intersection):
        target_child_frame = frame.copy()
        target_child_frame.point = self.intersection_closest_point
        frame_normal = intersection.direction
        target_child_frame.xaxis = frame_normal
        angle = Vector.angle_vectors([target_child_frame.normal], [frame_normal])
        R = Rotation.from_axis_and_angle(target_child_frame.yaxis, *angle, target_child_frame.point)
        r_frame = target_child_frame.transformed(R)
        r_frame.point += r_frame.yaxis * self.depth/2
        r_frame.point += r_frame.normal * self.depth
        r_frame.point += -r_frame.xaxis * self.offset_target_child
        axis = Line.from_point_and_vector(r_frame.point, r_frame.xaxis * self.stick_length)
        z_vector = r_frame.yaxis
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)
        return new_stick

    def visualize(self):
        return [stick.geometry for stick in self.sticks]