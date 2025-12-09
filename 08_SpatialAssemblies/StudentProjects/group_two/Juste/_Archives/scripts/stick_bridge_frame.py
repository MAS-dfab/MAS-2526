from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Plane
from compas.geometry import Rotation
from compas.geometry import closest_point_on_line
import math

from single_stick import Stick

def compare_angles(frame_0, frame_1):
        #calculate angle between the normals of root frame and target frame
        normal_deviation = math.degrees(*Vector.angle_vectors([frame_0.normal], [frame_1.normal]))

        return normal_deviation

def get_plane_from_frame(frame):
        plane = Plane.from_frame(frame)
        plane.normal = frame.yaxis
        return plane

class BridgeIndex:
    def __init__(self, branches, segment_index_a, face_index_a, segment_index_b, face_index_b, bridge_a_offset=0.0, bridge_b_offset=0.0, stick_length=None, width=None, depth=None):
        """
        Constructor for BridgeIndex module
        
        Args:
            branches: input, list of existing branches ('sticks')

            segment_index_a: index of the branch segment A
            face_index_a: face index on branch A
            bridge_a_offset: offset of bridge_a stick along its xaxis

            segment_index_b: index of the branch segment B
            face_index_b: face index on branch B

            bridge_a_offset: offset of bridge_a stick along its xaxis (defaults to 0.0)
            bridge_b_offset: offset of bridge_b stick along its xaxis (defaults to 0.0)

            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """

        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.sticks = []

        # self.root_frame = root_frame
        # self.root_frame_axis = root_frame.xaxis * self.stick_length
        # self.target_frame = target_frame
        # self.target_frame_axis = target_frame.xaxis * self.stick_length

        # self.offset_root_child = offset_root_child
        # self.offset_target_child = offset_target_child

        """secondary properties"""
        #angle between root_frame and target_frame
        self.normal_deviation = self.compare_angles(self.root_frame, self.target_frame)

        #algine target_frame to the 'orientation' of root frame
        self.rotated_target_frame = self.rotate_target_frame(self.target_frame)

        #child (secondary) frame of root and target frame
        self.root_child_frame = self.get_root_child_frame(self.root_frame, self.root_frame_axis)
        self.target_child_frame = self.get_target_child_frame(self.rotated_target_frame, self.target_frame_axis)

        #find intersection (line) of two frames based on their planes
        self.frame_intersection = self.get_frame_intersection(self.root_child_frame, self.target_child_frame)
        self.intersection_closest_point = self.get_intersection_closest_point(self.root_child_frame, self.frame_intersection)

        #create bridging sticks
        self.root_child_stick = self.get_root_child_stick(self.root_child_frame, self.frame_intersection)
        self.target_child_stick = self.get_target_child_stick(self.target_child_frame, self.frame_intersection)

    def compare_angles(self, frame_0, frame_1):
        #calculate angle between the normals of root frame and target frame
        normal_deviation = math.degrees(*
            Vector.angle_vectors([frame_0.normal], [frame_1.normal]))

        return normal_deviation
    
    def rotate_target_frame(self,target_frame):
        # rotate target 90 degrees if the deviation is greater than 90
        if self.normal_deviation > 180:
             R_2 = Rotation.from_axis_and_angle(target_frame.axis, - math.pi, target_frame.point)
             rotated_frame = target_frame.transformed(R_2)
        
        elif self.normal_deviation > 90:
            R = Rotation.from_axis_and_angle(target_frame.xaxis, - math.pi / 2, target_frame.point)
            rotated_frame = target_frame.transformed(R)

        else:
            rotated_frame = target_frame.copy()

        return rotated_frame
    
    def get_root_child_frame(self, frame, axis, flip = False):
        #create a child frame based on the root frame
        child_frame = frame.copy()

        #translate child frame to the middle of root frame
        axis_mid = Line.from_point_and_vector(child_frame.point, axis).midpoint
        child_frame.point = axis_mid

        #choose face index 1 or 3
        if flip == False:
             face_index = 1
        
        elif flip == True:
             face_index = 3

        #rotate frame and translate to face
        angle = face_index * math.pi/2
        R = Rotation.from_axis_and_angle(child_frame.xaxis, angle, child_frame.point)
        new_frame = child_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth/2

        #offset frame
        return new_frame
    
    def get_target_child_frame(self, frame, axis, flip = False):
        #create a child frame based on the root frame
        child_frame = frame.copy()

        #translate child frame to the middle of root frame
        axis_mid = Line.from_point_and_vector(child_frame.point, axis).midpoint
        child_frame.point = axis_mid

        #choose face index 0 or 2
        if flip == False:
             face_index = 1
        
        elif flip == True:
             face_index = 3

        #rotate frame and translate to face
        angle = face_index * math.pi/2
        R = Rotation.from_axis_and_angle(child_frame.xaxis, angle, child_frame.point)
        new_frame = child_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth/2

        #offset frame
        return new_frame

    def get_frame_intersection(self, frame_0, frame_1):
        #plane based on frame
        plane_0 = Plane.from_frame(frame_0)
        plane_0.normal = frame_0.yaxis

        #plane based on frame
        plane_1 = Plane.from_frame(frame_1)
        plane_1.normal = frame_1.yaxis

        #find the intersection line between the two planes
        int_line = plane_0.intersection_with_plane(plane_1)
        return int_line
    
    def get_intersection_closest_point(self, frame, intersection):
        #find closest point on line from frame
        point = Point(*closest_point_on_line(frame.point, intersection))
        return point
    
    def get_root_child_stick(self, frame, intersection):
        #copy input frame
        root_child_frame = frame.copy()
        
        #find end point along intersection. closest point finds perpendicular line.
        vector_end_point = Point(*closest_point_on_line(root_child_frame.point, intersection))

        #create new axis towards vector_end_point
        root_child_axis_vector = Vector.from_start_end(root_child_frame.point, vector_end_point)
        root_child_axis = root_child_axis_vector.unitized()

        #offset frame point to surface of stick
        root_child_frame.point += root_child_frame.yaxis * self.depth/2

        #offset frame point along length of stick
        root_child_frame.point += -root_child_axis * self.offset_root_child

        #create stick properties
        axis = Line.from_point_and_vector(root_child_frame.point, root_child_axis * self.stick_length)
        z_vector = root_child_frame.yaxis

        #initialise new stick
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)

        return new_stick
    
    def get_target_child_stick(self, frame, intersection): #intersection is a compas Line
        #copy input frame
        target_child_frame = frame.copy()

        #move frame to intersection closest point. closest point to find perpendicular line.
        target_child_frame.point = self.intersection_closest_point

        #align frame normal to make co-planar
        frame_normal = intersection.direction #returns unit vector parallel to intersectin line
        target_child_frame.xaxis = frame_normal

        #rotate frame to align in the direction of root child axis
        angle = Vector.angle_vectors([target_child_frame.normal], [frame_normal])
        R = Rotation.from_axis_and_angle(target_child_frame.yaxis, *angle, target_child_frame.point)
        r_target_child_frame = target_child_frame.transformed(R)
        
        #offset frame point to surface of stick
        r_target_child_frame.point += r_target_child_frame.yaxis * self.depth/2
        r_target_child_frame.point += r_target_child_frame.normal * self.depth

        #offset frame point along length of stick
        r_target_child_frame.point += -r_target_child_frame.xaxis * self.offset_target_child

        #create stick properties
        axis = Line.from_point_and_vector(r_target_child_frame.point, r_target_child_frame.xaxis * self.stick_length)
        z_vector = r_target_child_frame.yaxis

        #initialise new stick
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)
        return new_stick
        
    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]