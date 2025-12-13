from compas.geometry import Line
from compas.geometry import Point
from compas.geometry import Vector
from compas.geometry import Plane
from compas.geometry import Rotation
from compas.geometry import Translation
from compas.geometry import closest_point_on_line
from compas.geometry import distance_point_point
import math

from stick_axis import Stick

"""
To implement:
    fix bug on mirror root (rotate the target_child_stick to sit between root stick and target stick)
    align child sticks to sit in the middle, before offset.
"""

def compare_angles(frame_0, frame_1):
        #calculate angle between the normals of root frame and target frame
        normal_deviation = math.degrees(*Vector.angle_vectors([frame_0.normal], [frame_1.normal]))

        return normal_deviation

def get_plane_from_frame(frame):
        plane = Plane.from_frame(frame)
        plane.normal = frame.yaxis
        return plane

class BridgeIndex:
    def __init__(self, frames_list, xaxis_list, root_index, target_index, 
                root_dummy_face_index, target_dummy_face_index,
                root_dummy_length = 0.0, root_dummy_offset = 0.0,
                target_dummy_length = 0.0, target_dummy_offset = 0.0,
                root_offset=0.0, root_child_offset=0.0, target_child_offset=0.0, 
                stick_length=None, width=None, depth=None,
                solution=True, mirror_root=True, mirror_target=True
                ):
        """
        Constructor for BridgeIndex module
        
        Args:
            branches: input, list of existing branches ('sticks')

            root_index: the index of the stick within the list of 'branches' to become the 'root' stick
            target_index: the index of the stick within the list of 'branches' to become the 'target' stick

            root_offset: the offset of the 'dummy root stick' along the root stick's xaxis
            target_offset: the offset of the 'dummy target stick' along the target stick's xaxis

            root_child_offset: the offset of the 'root child stick' along its own xaxis
            target_child_offset: the offset of the 'target child stick' along its own xaxis

            solution: toggle between 2 solutions
            mirror_root: toggle between the location of root_child_stick between 2 opposite faces on root_stick
            mirror_target: toggle between the location of target_child_stick between 2 opposite faces on target_stick

            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """

        self.root_index = root_index
        self.target_index = target_index

        self.root_dummy_face_index = root_dummy_face_index
        self.target_dummy_face_index = target_dummy_face_index

        self.root_dummy_length = root_dummy_length
        self.root_dummy_offset = root_dummy_offset
        
        self.target_dummy_length = target_dummy_length
        self.target_dummy_offset = target_dummy_offset

        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_offset = root_offset

        #boolean containers
        self.solution = solution
        self.mirror_root = mirror_root
        self.mirror_target = mirror_target

        #lists containers
        self.frames_list = [f.copy() for f in frames_list]
        #convert xaxis_list into vector list
        self.xaxis_list = [x.copy() for x in xaxis_list]
        self.sticks = []

        #identify root and target frame from specified indexes
        self.root_frame = self.frames_list[self.root_index].copy()
        self.root_frame_axis = self.xaxis_list[self.root_index].copy()
        self.root_axis_length = self.root_frame_axis.length

        self.target_frame = self.frames_list[self.target_index].copy()
        self.target_frame_axis = self.xaxis_list[self.target_index].copy()
        self.target_axis_length = self.target_frame_axis.length

        self.offset_root_child = root_child_offset
        self.offset_target_child = target_child_offset

        """secondary properties"""
        #angle between root_frame and target_frame
        self.normal_deviation = self.compare_angles(self.root_frame, self.target_frame)

        #align target_frame to the 'orientation' of root frame
        self.rotated_target_frame = self.rotate_target_frame(self.target_frame)

        #child (secondary) frame of root and target frame
        self.root_child_frame = self.get_root_child_frame(self.root_frame, self.root_frame_axis, self.solution, self.mirror_root)
        self.target_child_frame = self.get_target_child_frame(self.rotated_target_frame, self.target_frame_axis, self.solution, self.mirror_target)

        #dummy (duplicate) sticks of root frame
        self.root_dummy_stick = self.get_root_dummy_stick(self.root_frame, self.root_dummy_face_index, self.root_dummy_length, self.root_dummy_offset)

        #find intersection (line) of two frames based on their planes
        self.frame_intersection = self.get_frame_intersection(self.root_child_frame, self.target_child_frame)
        self.intersection_closest_point = self.get_intersection_closest_point(self.root_child_frame, self.frame_intersection)

        #create bridging sticks
        self.root_child_stick = self.get_root_child_stick(self.root_child_frame, self.frame_intersection)
        self.target_child_stick = self.get_target_child_stick(self.target_child_frame, self.frame_intersection)

        #dummy (duplicate) sticks of target frame
        self.target_dummy_stick = self.get_target_dummy_stick(self.target_frame, self.target_dummy_face_index, self.target_dummy_length, self.target_dummy_offset)

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

    def get_root_dummy_stick(self, frame, face_index, length, offset):
        #create a dummy frame based on the root frame
        dummy_frame = frame.copy()

        angle = face_index * math.pi/2
        R = Rotation.from_axis_and_angle(dummy_frame.xaxis, angle, dummy_frame.point)
        new_frame = dummy_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth
        
        #offset along new_frame.xaxis
        new_frame.point += new_frame.xaxis * offset

        #create stick properties
        axis = Line.from_point_and_vector(new_frame.point, new_frame.xaxis * length)
        z_vector = new_frame.yaxis

        #initialise new stick
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)

        return new_stick

    def get_root_child_frame(self, frame, axis, solution, flip):
        #create a child frame based on the root frame
        child_frame = frame.copy()

        #translate child frame to the middle of root frame
        axis_mid = Line.from_point_and_vector(child_frame.point, axis).midpoint
        child_frame.point = axis_mid

        #translate child_frame along root_axis
        direction = self.root_frame_axis.unitized()
        T = Translation.from_vector(direction * self.root_offset)
        translated_c_frame = child_frame.transformed(T)

        #choose face index 1 or 3
        if solution == True and flip == True:
             face_index = 0
        
        elif solution == True and flip == False:
             face_index = 2

        elif solution == False and flip == True:
             face_index = 1
        
        elif solution == False and flip == False:
             face_index = 3

        #rotate frame and translate to face
        angle = face_index * math.pi/2
        R = Rotation.from_axis_and_angle(translated_c_frame.xaxis, angle, translated_c_frame.point)
        new_frame = translated_c_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth/2

        #offset frame
        return new_frame

    def get_target_dummy_stick(self, frame, face_index, length, offset):
        #create a dummy frame based on the root frame
        dummy_frame = frame.copy()

        angle = face_index * math.pi/2
        R = Rotation.from_axis_and_angle(dummy_frame.xaxis, angle, dummy_frame.point)
        new_frame = dummy_frame.transformed(R)
        new_frame.point += new_frame.yaxis * self.depth

        #offset along new_frame.xaxis
        new_frame.point += new_frame.xaxis * offset

        #create stick properties
        axis = Line.from_point_and_vector(new_frame.point, new_frame.xaxis * length)
        z_vector = new_frame.yaxis

        #initialise new stick
        new_stick = Stick(axis, z_vector)
        self.sticks.append(new_stick)

        return new_stick
 
    def get_target_child_frame(self, frame, axis, solution, flip):
        #create a child frame based on the root frame
        child_frame = frame.copy()

        #translate child frame to the middle of root frame
        axis_mid = Line.from_point_and_vector(child_frame.point, axis).midpoint
        child_frame.point = axis_mid

        #choose face index 0 or 2
        if solution == True and flip == True:
             face_index = 0
        
        elif solution == True and flip == False:
             face_index = 2

        elif solution == False and flip == True:
             face_index = 1

        elif solution == False and flip == False:
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

        #offset frame point to fit in middle
        middle_offset = (self.stick_length - root_child_axis_vector.length)/2

        #offset frame point along length of stick
        root_child_frame.point += -root_child_axis * (self.offset_root_child + middle_offset)

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
        target_frame = frame.copy()

        #move frame to intersection closest point. closest point to find perpendicular line.
        target_child_frame.point = self.intersection_closest_point

        #align frame normal to make co-planar
        frame_normal = intersection.direction #returns unit vector parallel to intersection line
        target_child_frame.xaxis = frame_normal

        #rotate frame to align in the direction of root child axis
        angle = Vector.angle_vectors([target_child_frame.normal], [frame_normal])

        R = Rotation.from_axis_and_angle(target_child_frame.yaxis, *angle, target_child_frame.point)
        r_target_child_frame = target_child_frame.transformed(R)
        
        #offset frame point to surface of stick
        r_target_child_frame.point += r_target_child_frame.yaxis * self.depth/2
        r_target_child_frame.point += r_target_child_frame.normal * self.depth

        #find distance between intersection point and frame
        length = distance_point_point(target_child_frame.point, target_frame.point)
        middle_offset = (self.stick_length - length)/2
        #offset frame point along length of stick
        if self.mirror_root == False:
            r_target_child_frame.point += -r_target_child_frame.xaxis * (self.offset_target_child - middle_offset + length)
        elif self.mirror_root == True:
            r_target_child_frame.point += -r_target_child_frame.xaxis * (self.offset_target_child + middle_offset + length)

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