from compas.geometry import Line, Vector, Point, Rotation, Frame
import math
from stick import Stick
import copy

from compas.geometry import angle_vectors_projected


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

class BranchingModule:
    def __init__(self, root_frame, stick_length=None, width=None, depth=None):
        """
        Constructor for Branching module.
        
        Args:
            root_frame: Frame from which tree will grow
            stick_length: Length of each stick
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """
        self.root_frame = root_frame
        self.sticks = []
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH  
        self._init_first_stick(root_frame)
    
    def _init_first_stick(self, frame):
        """
        Private method for creating the first stick.
        
        Args:
            frame: Frame from which stick will grow
        """
        start_point = frame.point
        end_point = start_point + frame.zaxis * self.stick_length
        axis = Line(start_point, end_point)
        stick = Stick(axis, self.width, self.depth)
        self.sticks.append(stick)

    
    def get_face_frame(self, stick_index, face_index):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            stick_index: Index of the stick
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face
        """
        stick = self.sticks[stick_index]
        
        frame = stick.frame.copy()
        frame.point = stick.axis.end
                
        # Rotate around stick axis to get different faces
        angle = (face_index % 4) * math.pi / 2
        R = Rotation.from_axis_and_angle(frame.xaxis, angle, stick.axis.end)
        r_frame = frame.transformed(R)
        
        # Offset frame to the face surface
        face_normal = r_frame.yaxis
        r_frame.point = r_frame.point + face_normal * stick.width     
        return r_frame
    
    def grow_stick(self, from_stick_index=-1, face_index=0, offset=10, angle=0):
        """
        Grows a new stick from an existing stick.
        
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            offset: Offset along the face normal
            angle: Angle of rotation in radians
        """
        
        # Get face frame
        face_frame = self.get_face_frame(from_stick_index, face_index).copy()
        
        # Apply offset along face normal (zaxis of face frame)
        face_frame.point = face_frame.point + face_frame.xaxis * -offset
        
        # Rotate around the face's yaxis (perpendicular to both face normal and stick axis)
        if angle != 0:
            R = Rotation.from_axis_and_angle(face_frame.yaxis, angle, face_frame.point)
            face_frame.transform(R)


        face_frame.point = face_frame.point + face_frame.xaxis * -offset
        
        # Create new stick
        start_point = face_frame.point
        end_point = start_point + face_frame.xaxis * self.stick_length
        axis = Line(start_point, end_point)
        new_stick = Stick(axis, self.width, self.depth, zvector=face_frame.yaxis)
        
        self.sticks.append(new_stick)

    
    def grow_stick_towards_auto(self, target_point, from_stick_index=-1, offset=10):
        """
        Automatically selects the best face and grows towards target.
        
        Args:
            target_point: Point to grow towards
            from_stick_index: Index of stick to grow from (-1 = last stick)
            offset: Offset along the face normal
            
        Returns:
            bool: True if stick endpoint is within proximity of target
        """
        # Find best face pointing towards target
        stick = self.sticks[from_stick_index]
        target_dir = Vector.from_start_end(stick.axis.end, target_point)
        target_dir.unitize()
        
        best_face = 0
        best_dot = -float('inf')
        
        for i in range(4):
            face_frame = self.get_face_frame(from_stick_index, i)
            test_dir = face_frame.zaxis
            dot = target_dir.dot(test_dir)
            if dot > best_dot:
                best_dot = dot
                best_index = i
                best_face = face_frame
        
        # Calculate angle to target
        best_face.point = best_face.point + best_face.xaxis * -offset
        
        target_vector = Vector.from_start_end(best_face.point, target_point)

        angle = 0
        if target_vector.length > 0.001:
            target_vector.unitize()
            angle = angle_vectors_projected(best_face.xaxis, target_vector, best_face.yaxis)
        
        # Grow stick with calculated angle
        self.grow_stick(from_stick_index, best_index, offset, angle)
        
        # Check if reached target
        last_stick = self.sticks[-1]
        distance = Point(*target_point).distance_to_point(Point(*last_stick.axis.end))
        reached = distance < (self.stick_length * 0.5)
        
        return reached
      
    
    def grow_towards_frame(self, target_frame, offset = 20.0, from_stick_index=-1, face_index=0, n_joints=2):
        """
        Grows a chain of sticks towards a target frame using analytical IK.
        Each joint rotates around previous stick's y-axis.
        
        Args:
            target_frame: Target frame (end effector goal)
            from_stick_index: Index of stick to grow from (-1 = last stick)
            face_index: Face index (0-3) to start from
            n_joints: Number of joints/sticks to create
        """
        from compas.geometry import distance_point_point
        
        face_frame = self.get_face_frame(from_stick_index, face_index)

        # Calculate distance to target from face frame
        target_point = target_frame.point
        
        total_angle = angle_vectors_projected(-face_frame.xaxis, target_frame.zaxis, face_frame.yaxis)
        
        # Distribute angle across joints (first joint gets 10% of total)
        angles = [total_angle * 0.1] + [total_angle * 0.9 / (n_joints - 1) for _ in range(n_joints - 1)]
        
        for i in range(n_joints):
            # Apply offset along negative xaxis (inward, like grow logic)
            if i>0:
                face_frame = self.get_face_frame(from_stick_index, face_index)
            current_frame = face_frame.copy()
            offset_frame = current_frame.copy()
            offset_frame.point = offset_frame.point + offset_frame.xaxis * -offset
            
            # Apply joint angle (rotation around y-axis)
            angle = angles[i]
            R_joint = Rotation.from_axis_and_angle(offset_frame.yaxis, angle, offset_frame.point)
            rotated_frame = offset_frame.transformed(R_joint)
            
            # Apply offset again after rotation (like in grow_stick)
            rotated_frame.point = rotated_frame.point + rotated_frame.xaxis * -offset
            
            # Create stick in rotated direction with calculated length
            distance_to_target = distance_point_point(rotated_frame.point, target_point)
            link_length = distance_to_target

            end_point = rotated_frame.point + rotated_frame.xaxis * link_length
            stick_axis = Line(rotated_frame.point, end_point)
            
            new_stick = Stick(stick_axis, self.width, self.depth, zvector=rotated_frame.yaxis)
            
            self.sticks.append(new_stick)


    
    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]




def generate_robotic_arm_ik(base_frame, target_frame, stick_width, stick_depth, n_joints=2, face_index=0):
    """
    Generates a robotic arm (chain of sticks) from base to target using analytical IK.
    Uses a simple 2-link analytical solution extended to n-joints.
    Each joint is revolute (rotates around previous stick's y-axis).
    
    Args:
        base_frame: Starting frame (base of robot/stick)
        target_frame: Target frame (end effector goal)
        stick_width: Width of sticks
        stick_depth: Depth of sticks
        n_joints: Number of joints/sticks in the arm
        face_index: Face index (0-3) to start from on base stick
        
    Returns:
        list: List of Stick objects forming the arm
    """
    from compas.geometry import angle_vectors_signed, distance_point_point, Frame
    import math
    
    # Get face frame from base frame (treat base_frame like a stick frame)
    # Rotate around xaxis to get different faces
    angle_face = (face_index % 4) * math.pi / 2
    R_face = Rotation.from_axis_and_angle(base_frame.xaxis, angle_face, base_frame.point)
    face_frame = base_frame.transformed(R_face)
    
    # Calculate distance to target from face frame
    target_point = target_frame.point
    distance = distance_point_point(face_frame.point, target_point)
    
    # Each link length (variable to reach target)
    link_length = distance / n_joints
    
    # Direction from face to target
    direction_to_target = target_point - face_frame.point
    direction_to_target = direction_to_target.unitized()
    
    # Calculate angle between face direction and target direction
    face_direction = face_frame.xaxis
    total_angle = angle_vectors_signed(face_direction, direction_to_target, face_frame.yaxis, deg=False)
    
    # Distribute angle across joints
    angles = [total_angle / n_joints for _ in range(n_joints)]
    
    # Build the arm with offset similar to grow logic
    sticks = []
    current_frame = face_frame.copy()
    
    # Calculate offset (half the stick width to create gap between segments)
    offset = stick_width / 2
    
    for i in range(n_joints):
        # Apply offset along negative xaxis (inward, like grow logic)
        offset_frame = current_frame.copy()
        offset_frame.point = offset_frame.point + offset_frame.xaxis * -offset
        
        # Apply joint angle (rotation around y-axis)
        angle = angles[i]
        R_joint = Rotation.from_axis_and_angle(offset_frame.yaxis, angle, offset_frame.point)
        rotated_frame = offset_frame.transformed(R_joint)
        
        # Apply offset again after rotation (like in grow_stick)
        rotated_frame.point = rotated_frame.point + rotated_frame.xaxis * -offset
        
        # Create stick in rotated direction with calculated length
        end_point = rotated_frame.point + rotated_frame.xaxis * link_length
        stick_axis = Line(rotated_frame.point, end_point)
        
        stick = Stick(stick_axis, stick_width, stick_depth, zvector=rotated_frame.yaxis)
        sticks.append(stick)
        
        # Update frame for next joint (at end of stick)
        current_frame = Frame(end_point, rotated_frame.xaxis, rotated_frame.yaxis)
    
    return sticks

