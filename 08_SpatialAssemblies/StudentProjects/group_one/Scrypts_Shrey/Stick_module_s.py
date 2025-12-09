from compas.geometry import Line, Frame, Vector, Rotation, Box, Point
import math
from Sticks_s import Stick

class OStickModule:
    def __init__(self, pt, stick_length, stick_width, stick_depth, offset=None):
        self.pt = pt
        self.length = stick_length
        self.width = stick_width
        self.depth = stick_depth
        self.offset = offset if offset is not None else stick_length  # Default to stick_length
        self.sticks = []  

    def create_orthogonal_module(self, type = {"x" : 0, "y" : 0, "z" : 0}):

        # stick x 
        offsetpt_x = (self.pt - Vector(self.depth/2,0,0) + Vector(0,2*self.depth*type["x"],0))
        
        px = offsetpt_x + Vector(self.length, 0, 0)      
        stick_x = Stick(Line(offsetpt_x, px), self.width, self.depth)
        if type ["x"] != 2:
            self.sticks.append(stick_x)

        # stick y

        offsetpt_y = (self.pt 
                      - Vector(0,self.depth/2,0) 
                      + Vector (0,0,self.depth)
                      + Vector(2*self.depth*type["y"],0,0))
        py = offsetpt_y + Vector(0,self.length,0)
        stick_y = Stick(Line(offsetpt_y, py), self.width, self.depth)
        if type ["y"] != 2:
            self.sticks.append(stick_y)

        #stick z
        
        p0 = self.pt
        offsetpt_z = (self.pt
                      + Vector(0,self.depth,0)
                      + Vector(self.depth,0,0)
                      - Vector(0,0,self.depth/2)
                      - Vector(0,2*self.depth*type["z"],0))
        pz = offsetpt_z + Vector (0,0,self.length)
        stick_z = Stick(Line(offsetpt_z,pz), self.width, self.depth)
        if type ["z"] != 2:
            self.sticks.append(stick_z)
    
    
    
    def create_orthogonal_module_extended(self):

        # stick x 
        offsetpt_x = (self.pt - (self.depth*2,0,0))
        
        px = offsetpt_x + Vector(self.length, 0, 0)
        stick_x = Stick(Line(offsetpt_x, px), self.width, self.depth)
        self.sticks.append(stick_x)

        # stick y

        offsetpt_y = (self.pt 
                      - Vector(0,self.depth*2,0) 
                      + Vector (0,0,self.depth))
        py = offsetpt_y + Vector(0,self.length,0)
        stick_y = Stick(Line(offsetpt_y, py), self.width, self.depth)
        self.sticks.append(stick_y)

        #stick z
        
        p0 = self.pt
        offsetpt_z = (self.pt
                      +Vector(0,self.depth,0)
                      +Vector(self.depth,0,0)
                      -Vector(0,0,self.depth*2))
        pz = offsetpt_z + Vector (0,0,self.length)
        stick_z = Stick(Line(offsetpt_z,pz), self.width, self.depth)
        self.sticks.append(stick_z)

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
        # Draw line based on start frame

        stick_axis = Line.from_point_and_vector(frame.point, frame.zaxis*self.stick_length) 

        # Create stick 

        my_stick = Stick(stick_axis, z_vector= frame.yaxis) 

        # Add stick to list of sticks

        self.sticks.append(my_stick)

        #

    def get_face_frame(self, stick_index, face_index):
        """
        Gets a frame on one of the four faces of a stick.
        Args:
            stick_index: Index of the stick
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face
        """

        # Rotate stick frame based on index 

        stick_frame = self.sticks[stick_index].frame

        angle = face_index * math.radians(90) 
        
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle = angle, point = stick_frame.point)
        
        new_frame = stick_frame.transformed(R)

        new_frame.point = self.sticks[stick_index].axis.end

        # Offset frame to be on surface on stick

        new_frame.point += new_frame.yaxis * self.depth/2

        return new_frame
         
    def grow_stick(self, from_stick_index = -1, face_index = 0, angle = 0.0, offset = 10):
        """
        Grows a new stick from an existing stick.
        
        Args:
            from_stick_index: Index of stick to grow from 
            face_index: Index of the face to grow from (0-3)
            angle: Angle of rotation in radians
        """
                
        # Get position on original stick
        position = self.get_face_frame(from_stick_index, face_index).copy()
        position.point += position.yaxis * self.depth/2
        position.point += position.xaxis * -offset


        # Rotate along face frame

        R = Rotation.from_axis_and_angle(position.yaxis, math.radians(angle), point= position.point)

        position.transform(R)

        # offset along stick lenght

        position.point += position.xaxis * -offset
            
        # Create new stick
        centerline = Line.from_point_and_vector(position.point, position.xaxis * self.stick_length)
        zvector = position.yaxis
        new_stick = Stick(centerline, zvector)
        self.sticks.append(new_stick)


    def visualize(self):
        """
        Returns all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
    
    
    
class SquareModule:
    def __init__(self, root_frame, stick_length=200, width=None, depth=None):
        """
        Creates two perpendicular horizontal sticks from a frame.
        
        Args:
            root_frame: Frame from which sticks will be created
            stick_length: Length of each stick (default 200)
            width: Width of sticks (defaults to Stick.Width)
            depth: Depth of sticks (defaults to Stick.Depth)
        """
        self.root_frame = root_frame
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.sticks = []
        self._init_sticks(root_frame)
    
    def _init_sticks(self, frame):
        """
        Creates two perpendicular sticks.
        
        Args:
            frame: Frame from which sticks will be created
        """
        # First stick along frame's x-axis
        stick_axis_1 = Line.from_point_and_vector(frame.point, frame.xaxis * self.stick_length)
        stick_1 = Stick(stick_axis_1, self.width, self.depth)
        self.sticks.append(stick_1)
        
        # Second stick along frame's y-axis (90 degrees rotated)
        stick_axis_2 = Line.from_point_and_vector(frame.point + Vector(self.width/2,self.width/2,0), frame.yaxis * self.stick_length)
        stick_2 = Stick(stick_axis_2, self.width, self.depth)
        self.sticks.append(stick_2)

        # Third stick along frame's x-axis (90 degrees rotated)
        stick_axis_3 = Line.from_point_and_vector(stick_2.axis.end + Vector((self.width/2),-(self.width/2),0), frame.xaxis * self.stick_length)
        stick_3 = Stick(stick_axis_3, self.width, self.depth)
        self.sticks.append(stick_3)
        
        # Fourht stick along frame's y-axis (90 degrees rotated)
        stick_axis_4 = Line.from_point_and_vector(stick_3.axis.end + Vector(-(self.width/2),-(self.width/2),0), -frame.yaxis * self.stick_length)
        stick_4 = Stick(stick_axis_4, self.width, self.depth)
        self.sticks.append(stick_4)
    
    def visualize(self):
        """Returns all stick geometries."""
        return [stick.geometry for stick in self.sticks]
    

class CubeModule:
    def __init__(self, root_frame, stick_length=200, width=None, depth=None):
        """
        Creates a cube frame from sticks.
        
        Args:
            root_frame: Frame from which sticks will be created
            stick_length: Length of each stick (default 200)
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """
        self.root_frame = root_frame
        self.stick_length = stick_length
        self.depth = depth or Stick.DEPTH  
        self.width = width or Stick.WIDTH 
        self.sticks = []
        self._init_sticks(root_frame)
    
    def _init_sticks(self, frame):
        """
        Creates sticks forming a cube centered at the frame point.
        
        Args:
            frame: Frame from which sticks will be created
        """
        from compas.geometry import bounding_box, Translation
        
        # Calculate vertical stick length
        stick_vert_lenght = self.stick_length - (self.width*2) + self.depth
        
        # Temporarily create cube at origin to find its center
        temp_origin = frame.point
        
        # First stick along frame's x-axis
        stick_axis_1 = Line.from_point_and_vector(temp_origin, frame.xaxis * self.stick_length)
        stick_1 = Stick(stick_axis_1, width=self.width, depth=self.depth)
        self.sticks.append(stick_1)
        
        # Second stick along frame's y-axis
        stick_axis_2 = Line.from_point_and_vector(temp_origin + Vector(self.depth/2, self.depth/2, 0), frame.yaxis * self.stick_length)
        stick_2 = Stick(stick_axis_2, width=self.width, depth=self.depth)
        self.sticks.append(stick_2)

        # Third stick along frame's x-axis
        stick_axis_3 = Line.from_point_and_vector(stick_2.axis.end + Vector((self.depth/2), -(self.depth/2), 0), frame.xaxis * self.stick_length)
        stick_3 = Stick(stick_axis_3, width=self.width, depth=self.depth)
        self.sticks.append(stick_3)
        
        # Fourth stick along frame's y-axis
        stick_axis_4 = Line.from_point_and_vector(stick_3.axis.end + Vector(-(self.depth/2), -(self.depth/2), 0), -frame.yaxis * self.stick_length)
        stick_4 = Stick(stick_axis_4, width=self.width, depth=self.depth)
        self.sticks.append(stick_4)
        
        # Fifth stick along frame's z-axis
        stick_axis_5 = Line.from_point_and_vector(temp_origin + Vector(self.width/2, 0, self.width/2), frame.zaxis * stick_vert_lenght)
        stick_5 = Stick(stick_axis_5, width=self.width, depth=self.depth)
        self.sticks.append(stick_5)
        
        # Sixth stick along frame's z-axis
        stick_axis_6 = Line.from_point_and_vector(stick_2.axis.end + Vector((self.width/2) - (self.depth/2), -(self.depth/2), (self.width/2)), frame.zaxis * stick_vert_lenght)
        stick_6 = Stick(stick_axis_6, width=self.width, depth=self.depth)
        self.sticks.append(stick_6)

        # Seventh stick along frame's z-axis
        stick_axis_7 = Line.from_point_and_vector(stick_3.axis.end + Vector(-(self.width/2), 0, self.width/2), frame.zaxis * stick_vert_lenght)
        stick_7 = Stick(stick_axis_7, width=self.width, depth=self.depth)
        self.sticks.append(stick_7)

        # Eighth stick along frame's z-axis
        stick_axis_8 = Line.from_point_and_vector(stick_4.axis.end + Vector((self.depth/2)-(self.width/2), self.depth/2, self.width/2), frame.zaxis * stick_vert_lenght)
        stick_8 = Stick(stick_axis_8, width=self.width, depth=self.depth)
        self.sticks.append(stick_8)

        # Z offset for top layer
        z_offset = Vector(0, 0, stick_vert_lenght + self.width)
        
        # Ninth stick along frame's x-axis (same as stick 1, moved up)
        stick_axis_9 = Line.from_point_and_vector(temp_origin + z_offset, frame.xaxis * self.stick_length)
        stick_9 = Stick(stick_axis_9, width=self.width, depth=self.depth)
        self.sticks.append(stick_9)
        
        # Tenth stick along frame's y-axis (same as stick 2, moved up)
        stick_axis_10 = Line.from_point_and_vector(temp_origin + Vector(self.depth/2, self.depth/2, 0) + z_offset, frame.yaxis * self.stick_length)
        stick_10 = Stick(stick_axis_10, width=self.width, depth=self.depth)
        self.sticks.append(stick_10)

        # Eleventh stick along frame's x-axis (same as stick 3, moved up)
        stick_axis_11 = Line.from_point_and_vector(stick_10.axis.end + Vector((self.depth/2), -(self.depth/2), 0), frame.xaxis * self.stick_length)
        stick_11 = Stick(stick_axis_11, width=self.width, depth=self.depth)
        self.sticks.append(stick_11)
        
        # Twelfth stick along frame's y-axis (same as stick 4, moved up)
        stick_axis_12 = Line.from_point_and_vector(stick_11.axis.end + Vector(-(self.depth/2), -(self.depth/2), 0), -frame.yaxis * self.stick_length)
        stick_12 = Stick(stick_axis_12, width=self.width, depth=self.depth)
        self.sticks.append(stick_12)
        
        # Collect all vertices from all sticks
        all_points = []
        for stick in self.sticks:
            box = stick.geometry
            # Get the 8 vertices of the box
            all_points.extend(box.points)
        
        # Calculate bounding box and find its center
        bbox = bounding_box(all_points)
        bbox_center = sum(bbox, Vector(0, 0, 0)) / len(bbox)
        
        # Calculate offset needed to center at frame point
        offset = frame.point - bbox_center
        
        # Apply offset to all sticks
        T = Translation.from_vector(offset)
        for stick in self.sticks:
            stick.axis.transform(T)
            stick.frame.transform(T)

    def create_nested_cubes(self, num_cubes=2, scale_factors=None, scale_spacing=None):
        """
        Creates multiple nested cubes with different scale factors, all centered at the same point.
        
        Args:
            num_cubes: Number of nested cubes to create (default 2)
            scale_factors: List of scale factors for each cube. If None, automatically generates
                        factors starting from 1.0 with custom or default spacing
            scale_spacing: Spacing between scale factors (default 0.5 if None)
        
        Returns:
            List of lists, where each inner list contains Stick objects for one nested cube
        """
        from compas.geometry import bounding_box, Translation
        
        # Generate scale factors if not provided
        if scale_factors is None:
            if scale_spacing is None:
                scale_spacing = 0.5  # Default spacing
            
            # Create scale factors starting from 1.0, incrementing by scale_spacing
            scale_factors = [1.0 + (i * scale_spacing) for i in range(num_cubes)]
        
        # Get the center of the original cube
        original_points = []
        for stick in self.sticks:
            box = stick.geometry
            original_points.extend(box.points)
        original_bbox = bounding_box(original_points)
        original_center = sum(original_bbox, Vector(0, 0, 0)) / len(original_bbox)
        
        # Store the center for later use
        self.bbox_center = original_center
        
        all_nested_cubes = []
        
        # Create each nested cube
        for scale_factor in scale_factors[:num_cubes]:
            # Calculate new stick length
            new_stick_length = self.stick_length * scale_factor
            
            # Calculate vertical stick length for this cube
            stick_vert_lenght = new_stick_length - (self.width*2) + self.depth
            
            # Temporarily create cube at origin
            temp_origin = self.root_frame.point
            inner_sticks = []
            
            # First stick along frame's x-axis
            stick_axis_1 = Line.from_point_and_vector(temp_origin, self.root_frame.xaxis * new_stick_length)
            stick_1 = Stick(stick_axis_1, width=self.width, depth=self.depth)
            inner_sticks.append(stick_1)
            
            # Second stick along frame's y-axis
            stick_axis_2 = Line.from_point_and_vector(temp_origin + Vector(self.depth/2, self.depth/2, 0), self.root_frame.yaxis * new_stick_length)
            stick_2 = Stick(stick_axis_2, width=self.width, depth=self.depth)
            inner_sticks.append(stick_2)

            # Third stick along frame's x-axis
            stick_axis_3 = Line.from_point_and_vector(stick_2.axis.end + Vector((self.depth/2), -(self.depth/2), 0), self.root_frame.xaxis * new_stick_length)
            stick_3 = Stick(stick_axis_3, width=self.width, depth=self.depth)
            inner_sticks.append(stick_3)
            
            # Fourth stick along frame's y-axis
            stick_axis_4 = Line.from_point_and_vector(stick_3.axis.end + Vector(-(self.depth/2), -(self.depth/2), 0), -self.root_frame.yaxis * new_stick_length)
            stick_4 = Stick(stick_axis_4, width=self.width, depth=self.depth)
            inner_sticks.append(stick_4)
            
            # Fifth stick along frame's z-axis
            stick_axis_5 = Line.from_point_and_vector(temp_origin + Vector(self.width/2, 0, self.width/2), self.root_frame.zaxis * stick_vert_lenght)
            stick_5 = Stick(stick_axis_5, width=self.width, depth=self.depth)
            inner_sticks.append(stick_5)
            
            # Sixth stick along frame's z-axis
            stick_axis_6 = Line.from_point_and_vector(stick_2.axis.end + Vector((self.width/2) - (self.depth/2), -(self.depth/2), (self.width/2)), self.root_frame.zaxis * stick_vert_lenght)
            stick_6 = Stick(stick_axis_6, width=self.width, depth=self.depth)
            inner_sticks.append(stick_6)

            # Seventh stick along frame's z-axis
            stick_axis_7 = Line.from_point_and_vector(stick_3.axis.end + Vector(-(self.width/2), 0, self.width/2), self.root_frame.zaxis * stick_vert_lenght)
            stick_7 = Stick(stick_axis_7, width=self.width, depth=self.depth)
            inner_sticks.append(stick_7)

            # Eighth stick along frame's z-axis
            stick_axis_8 = Line.from_point_and_vector(stick_4.axis.end + Vector((self.depth/2)-(self.width/2), self.depth/2, self.width/2), self.root_frame.zaxis * stick_vert_lenght)
            stick_8 = Stick(stick_axis_8, width=self.width, depth=self.depth)
            inner_sticks.append(stick_8)

            # Z offset for top layer
            z_offset = Vector(0, 0, stick_vert_lenght + self.width)
            
            # Ninth stick along frame's x-axis (same as stick 1, moved up)
            stick_axis_9 = Line.from_point_and_vector(temp_origin + z_offset, self.root_frame.xaxis * new_stick_length)
            stick_9 = Stick(stick_axis_9, width=self.width, depth=self.depth)
            inner_sticks.append(stick_9)
            
            # Tenth stick along frame's y-axis (same as stick 2, moved up)
            stick_axis_10 = Line.from_point_and_vector(temp_origin + Vector(self.depth/2, self.depth/2, 0) + z_offset, self.root_frame.yaxis * new_stick_length)
            stick_10 = Stick(stick_axis_10, width=self.width, depth=self.depth)
            inner_sticks.append(stick_10)

            # Eleventh stick along frame's x-axis (same as stick 3, moved up)
            stick_axis_11 = Line.from_point_and_vector(stick_10.axis.end + Vector((self.depth/2), -(self.depth/2), 0), self.root_frame.xaxis * new_stick_length)
            stick_11 = Stick(stick_axis_11, width=self.width, depth=self.depth)
            inner_sticks.append(stick_11)
            
            # Twelfth stick along frame's y-axis (same as stick 4, moved up)
            stick_axis_12 = Line.from_point_and_vector(stick_11.axis.end + Vector(-(self.depth/2), -(self.depth/2), 0), -self.root_frame.yaxis * new_stick_length)
            stick_12 = Stick(stick_axis_12, width=self.width, depth=self.depth)
            inner_sticks.append(stick_12)
            
            # Collect all vertices from this cube's sticks
            all_points = []
            for stick in inner_sticks:
                box = stick.geometry
                all_points.extend(box.points)
            
            # Calculate bounding box and find its center
            bbox = bounding_box(all_points)
            bbox_center = sum(bbox, Vector(0, 0, 0)) / len(bbox)
            
            # Calculate offset to align with original center
            offset = original_center - bbox_center
            
            # Apply offset to all sticks in this cube
            T = Translation.from_vector(offset)
            for stick in inner_sticks:
                stick.axis.transform(T)
                stick.frame.transform(T)
            
            all_nested_cubes.append(inner_sticks)
        
        # Store nested cubes for later reference
        self.nested_cubes = all_nested_cubes
        
        return all_nested_cubes

    def rotate_cube(self, cube_index, x_angle=0, y_angle=0, z_angle=0):
        """
        Rotates a specific cube around the bbox center.
        
        Args:
            cube_index: Index of the cube to rotate (0 = main cube, 1+ = nested cubes)
            x_angle: Rotation angle around X-axis in degrees
            y_angle: Rotation angle around Y-axis in degrees
            z_angle: Rotation angle around Z-axis in degrees
        """
        from compas.geometry import Rotation
        import math
        
        # Get the cube to rotate
        if cube_index == 0:
            cube_sticks = self.sticks
        else:
            cube_sticks = self.nested_cubes[cube_index - 1]
        
        # Get rotation center (bbox center)
        center = self.bbox_center
        
        # Apply rotations in order: X, Y, Z
        if x_angle != 0:
            R_x = Rotation.from_axis_and_angle(Vector(1, 0, 0), math.radians(x_angle), point=center)
            for stick in cube_sticks:
                stick.axis.transform(R_x)
                stick.frame.transform(R_x)
        
        if y_angle != 0:
            R_y = Rotation.from_axis_and_angle(Vector(0, 1, 0), math.radians(y_angle), point=center)
            for stick in cube_sticks:
                stick.axis.transform(R_y)
                stick.frame.transform(R_y)
        
        if z_angle != 0:
            R_z = Rotation.from_axis_and_angle(Vector(0, 0, 1), math.radians(z_angle), point=center)
            for stick in cube_sticks:
                stick.axis.transform(R_z)
                stick.frame.transform(R_z)

    def rotate_cube_random(self, cube_index, x_range=[-45, 45], y_range=[-45, 45], z_range=[-45, 45], seed=None):
        """
        Rotates a specific cube with random angles within specified ranges.
        
        Args:
            cube_index: Index of the cube to rotate (0 = main cube, 1+ = nested cubes)
            x_range: [min, max] rotation range for X-axis in degrees
            y_range: [min, max] rotation range for Y-axis in degrees
            z_range: [min, max] rotation range for Z-axis in degrees
            seed: Random seed for reproducibility. If None, uses random seed.
        
        Returns:
            tuple: (x_angle, y_angle, z_angle, seed_used) - the random angles and seed used
        """
        import random
        
        # If seed provided, use it; otherwise generate one
        if seed is not None:
            seed_used = seed
            random.seed(seed + cube_index)  # Add cube_index to vary rotation per cube
        else:
            seed_used = random.randint(0, 1000000)
            random.seed(seed_used + cube_index)
        
        x_angle = random.uniform(x_range[0], x_range[1])
        y_angle = random.uniform(y_range[0], y_range[1])
        z_angle = random.uniform(z_range[0], z_range[1])
        
        self.rotate_cube(cube_index, x_angle, y_angle, z_angle)
        
        return x_angle, y_angle, z_angle, seed_used

    def visualize_all(self):
        """
        Returns all geometries from main cube and nested cubes.
        
        Returns:
            List of all Box geometries
        """
        geometries = [stick.geometry for stick in self.sticks]
        
        if hasattr(self, 'nested_cubes'):
            for cube_sticks in self.nested_cubes:
                geometries.extend([stick.geometry for stick in cube_sticks])
        
        return geometries
        
    def visualize(self):
        """Returns all stick geometries."""
        return [stick.geometry for stick in self.sticks]
        
    def visualize_nested_only(self):
        """
        Returns only geometries from nested cubes (excludes main cube).
        
        Returns:
            List of Box geometries from nested cubes only
        """
        geometries = []
        
        if hasattr(self, 'nested_cubes'):
            for cube_sticks in self.nested_cubes:
                geometries.extend([stick.geometry for stick in cube_sticks])
        
        return geometries


class ExtendedCubeModule:
    def __init__(self, root_frame, stick_length=200, width=None, depth=None):
        """
        Creates an extended cube module with sticks along the x-axis, y-axis, and z-axis.
        
        Args:
            root_frame: Frame from which sticks will be created
            stick_length: Length of each stick (default 200)
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """
        self.root_frame = root_frame
        self.stick_length = stick_length
        self.depth = depth or Stick.DEPTH  
        self.width = width or Stick.WIDTH 
        self.sticks = []
        self._init_sticks(root_frame)
    
    def _init_sticks(self, frame):
        """
        Creates sticks along the x-axis, y-axis, and z-axis from the frame point.
        
        Args:
            frame: Frame from which sticks will be created
        """
        # Create stick along frame's x-axis
        stick_axis_x = Line.from_point_and_vector(frame.point, frame.xaxis * self.stick_length)
        stick_x = Stick(stick_axis_x, width=self.width, depth=self.depth)
        self.sticks.append(stick_x)
        
        # Create stick along frame's y-axis with offset
        # Offset: +1 width in z-axis, +2.5 depth in x-axis, -2.5 depth in y-axis
        start_point_y = (frame.point 
                        + frame.zaxis * self.width 
                        + frame.xaxis * (self.depth * 2.5) 
                        - frame.yaxis * (self.depth * 2.5))
        stick_axis_y = Line.from_point_and_vector(start_point_y, frame.yaxis * self.stick_length)
        stick_y = Stick(stick_axis_y, width=self.width, depth=self.depth)
        self.sticks.append(stick_y)
        
        # Create stick along frame's z-axis with offset
        # Offset: -1 depth in y-axis, +1.5 depth in x-axis, -(0.5 width + depth) in z-axis
        start_point_z = (frame.point 
                        - frame.yaxis * self.depth 
                        + frame.xaxis * (self.depth * 2 - self.width * 0.5)
                        - frame.zaxis * (self.width * 0.5 + self.depth))
        stick_axis_z = Line.from_point_and_vector(start_point_z, frame.zaxis * self.stick_length)
        stick_z = Stick(stick_axis_z, width=self.width, depth=self.depth)
        self.sticks.append(stick_z)
    
    def visualize(self):
        """Returns all stick geometries."""
        return [stick.geometry for stick in self.sticks]


class StickWithFrames:
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, center_frame, stick_length, width=None, depth=None):
        """
        Create a stick using a center frame.
        
        Parameters:
            center_frame: Frame at the center of the stick (xaxis = stick direction)
            stick_length: Length of the stick
            width: Width of the stick (defaults to Stick.WIDTH)
            depth: Depth of the stick (defaults to Stick.DEPTH)
        """
        self.center_frame = center_frame
        self.length = stick_length
        self.width = width or StickWithFrames.WIDTH
        self.depth = depth or StickWithFrames.DEPTH
        
        # Calculate axis from center frame
        half_length = self.length / 2
        self.axis = Line(
            center_frame.point - center_frame.xaxis * half_length,
            center_frame.point + center_frame.xaxis * half_length
        )
        
        # Store all face frames
        self.face_frames = self._calculate_face_frames()
    
    def _calculate_face_frames(self):
        """
        Calculate frames for all 6 faces of the stick.
        Face frame's yaxis always points OUTWARD from the stick surface.
        
        Face indexing:
        - Face 0: +Y face (top)
        - Face 1: -Y face (bottom)
        - Face 2: +Z face (right side)
        - Face 3: -Z face (left side)
        - Face 4: +X face (end, smallest face)
        - Face 5: -X face (start, smallest face)
        
        Returns:
            list: List of 6 Frame objects, one for each face
        """
        frames = []
        center = self.center_frame.point
        
        # Face 0: +Y face (top face) - yaxis points UP (+Y)
        face0_point = center + self.center_frame.yaxis * (self.width / 2)
        face0_frame = Frame(face0_point, self.center_frame.xaxis, self.center_frame.yaxis)
        frames.append(face0_frame)
        
        # Face 1: -Y face (bottom face) - yaxis points DOWN (-Y)
        face1_point = center - self.center_frame.yaxis * (self.width / 2)
        face1_frame = Frame(face1_point, self.center_frame.xaxis, -self.center_frame.yaxis)
        frames.append(face1_frame)
        
        # Face 2: +Z face (right side) - yaxis points RIGHT (+Z)
        face2_point = center + self.center_frame.zaxis * (self.depth / 2)
        face2_frame = Frame(face2_point, self.center_frame.xaxis, self.center_frame.zaxis)
        frames.append(face2_frame)
        
        # Face 3: -Z face (left side) - yaxis points LEFT (-Z)
        face3_point = center - self.center_frame.zaxis * (self.depth / 2)
        face3_frame = Frame(face3_point, self.center_frame.xaxis, -self.center_frame.zaxis)
        frames.append(face3_frame)
        
        # Face 4: +X face (end face, smallest) - yaxis points FORWARD (+X)
        face4_point = center + self.center_frame.xaxis * (self.length / 2)
        face4_frame = Frame(face4_point, self.center_frame.xaxis, self.center_frame.yaxis)
        frames.append(face4_frame)
        
        # Face 5: -X face (start face, smallest) - yaxis points BACKWARD (-X)
        face5_point = center - self.center_frame.xaxis * (self.length / 2)
        face5_frame = Frame(face5_point, -self.center_frame.xaxis, self.center_frame.yaxis)
        frames.append(face5_frame)
        
        return frames
    
    def get_face_frame(self, face_index):
        """
        Get the frame of a specific face.
        
        Parameters:
            face_index: Index of the face (0-5)
                0: +Y (top), 1: -Y (bottom)
                2: +Z (right), 3: -Z (left)
                4: +X (end), 5: -X (start)
        
        Returns:
            Frame: Frame at the center of the specified face (yaxis points outward)
        """
        if 0 <= face_index < 6:
            return self.face_frames[face_index]
        else:
            raise ValueError(f"Face index must be between 0 and 5, got {face_index}")
    
    @property
    def geometry(self):
        """Returns the box geometry of the stick."""
        box = Box(self.length, self.width, self.depth, self.center_frame)
        return box
    
    @property
    def frame(self):
        """Returns the center frame for compatibility."""
        return self.center_frame


def stick_bridge_from_faces(stick0, face_index0, stick1, face_index1, width=None, depth=None):
    """
    Bridge two StickWithFrames using their face frames with axis-aligned connection.
    Each bridge stick connects face-to-face with the next, no intersections.
    
    Parameters:
        stick0: First StickWithFrames
        face_index0: Face index on stick0 to connect from (0-5)
        stick1: Second StickWithFrames
        face_index1: Face index on stick1 to connect to (0-5)
        width: Width of bridge sticks (defaults to stick0.width)
        depth: Depth of bridge sticks (defaults to stick0.depth)
        
    Returns:
        list: List of StickWithFrames objects forming the bridge
    """
    if width is None:
        width = stick0.width
    if depth is None:
        depth = stick0.depth
    
    # Get face frames
    frame0 = stick0.get_face_frame(face_index0)
    frame1 = stick1.get_face_frame(face_index1)
    
    # Start point - offset outward from first face
    start_point = frame0.point + frame0.yaxis * (depth / 2)
    # End point - offset outward from second face  
    end_point = frame1.point + frame1.yaxis * (depth / 2)
    
    # Calculate the offset needed to bridge
    delta = end_point - start_point
    dx, dy, dz = delta.x, delta.y, delta.z
    
    bridge_sticks = []
    current_point = start_point.copy()
    
    # Order movements by magnitude (longest first)
    movements = [
        ('x', abs(dx), dx, Vector(1, 0, 0)),
        ('y', abs(dy), dy, Vector(0, 1, 0)),
        ('z', abs(dz), dz, Vector(0, 0, 1))
    ]
    movements.sort(key=lambda m: m[1], reverse=True)
    
    # Filter out zero-length movements
    movements = [(n, m, s, v) for n, m, s, v in movements if m > 0.001]
    
    for i, (axis_name, magnitude, signed_dist, axis_vector) in enumerate(movements):
        # Direction along this axis
        direction = axis_vector * (1 if signed_dist > 0 else -1)
        
        # For first stick: start from current point
        # For middle sticks: account for depth offset from previous stick's end face
        # For last stick: end at the final point
        
        if i == 0:
            # First stick - starts at current_point
            start_pt = current_point.copy()
        else:
            # Subsequent sticks - offset by depth from previous stick's end face
            start_pt = current_point + direction * (depth / 2)
        
        if i == len(movements) - 1:
            # Last stick - ends at end_point
            end_pt = end_point.copy()
            stick_length = (end_pt - start_pt).length
        else:
            # Not last stick - full magnitude minus depth for end face offset
            stick_length = magnitude - (depth / 2)
            end_pt = start_pt + direction * stick_length
        
        # Create center point for this bridge stick
        center_pt = start_pt + direction * (stick_length / 2)
        
        # Create frame for bridge stick (xaxis = stick direction)
        xaxis = direction
        
        # Choose yaxis perpendicular to xaxis
        if abs(xaxis.dot(Vector(0, 0, 1))) < 0.9:
            zaxis = Vector(0, 0, 1)
        else:
            zaxis = Vector(1, 0, 0)
        
        yaxis = zaxis.cross(xaxis).unitized()
        zaxis = xaxis.cross(yaxis).unitized()
        
        bridge_frame = Frame(center_pt, xaxis, yaxis)
        
        # Create the bridge stick
        bridge_stick = StickWithFrames(bridge_frame, stick_length, width=width, depth=depth)
        bridge_sticks.append(bridge_stick)
        
        # Update current point - move to the end face of this stick
        current_point = end_pt.copy()
    
    return bridge_sticks


class Module_1:
    def __init__(self, root_frame, stick_length=200, width=None, depth=None):
        """
        Creates a module with four sticks.
        
        Args:
            root_frame: Frame from which sticks will be created
            stick_length: Length of each stick (default 200)
            width: Width of sticks (defaults to Stick.WIDTH)
            depth: Depth of sticks (defaults to Stick.DEPTH)
        """
        self.root_frame = root_frame
        self.stick_length = stick_length
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.sticks = []
        self._init_sticks(root_frame)
    
    def _init_sticks(self, frame):
        """
        Creates four sticks: two horizontal along x-axis, one vertical along z-axis, and one along y-axis.
        
        Args:
            frame: Frame from which sticks will be created
        """
        # First stick along frame's x-axis
        stick_axis_1 = Line.from_point_and_vector(frame.point - Vector(0, self.depth, 0) - frame.xaxis * (self.width/2) - frame.xaxis * (self.stick_length/4), frame.xaxis * self.stick_length)
        stick_1 = Stick(stick_axis_1, width=self.width, depth=self.depth)
        self.sticks.append(stick_1)
        
        # Second stick moved along y-axis by depth amount
        start_point_2 = frame.point + frame.yaxis * self.depth - frame.xaxis * (self.width/2) - frame.xaxis * (self.stick_length/4)
        stick_axis_2 = Line.from_point_and_vector(start_point_2, frame.xaxis * self.stick_length)
        stick_2 = Stick(stick_axis_2, width=self.width, depth=self.depth)
        self.sticks.append(stick_2)
        
        # Third stick along z-axis
        start_point_3 = frame.point + (frame.xaxis * (self.width/2)) - (frame.zaxis * self.stick_length/4) - frame.xaxis * (self.width/2)
        stick_axis_3 = Line.from_point_and_vector(start_point_3, frame.zaxis * self.stick_length)
        stick_3 = Stick(stick_axis_3, width=self.width, depth=self.depth)
        self.sticks.append(stick_3)
        
        # Fourth stick along y-axis
        start_point_4 = frame.point - frame.xaxis * (self.depth/2) - frame.yaxis * (self.stick_length/4) + frame.zaxis * self.width - frame.xaxis * (self.width/2) - frame.yaxis * (self.depth/2)
        stick_axis_4 = Line.from_point_and_vector(start_point_4, frame.yaxis * self.stick_length)
        stick_4 = Stick(stick_axis_4, width=self.width, depth=self.depth)
        self.sticks.append(stick_4)
    
    def create_stacked_modules(self, num_modules=5, vertical_spacing=None, rotation_angles=None, offset_pattern="none"):
        """
        Creates multiple stacked Module_1 instances with interlocking arrangement.
        Each module maintains its exact structure, positioned to interlock perfectly.
        
        Args:
            num_modules: Number of modules to stack (default 5)
            vertical_spacing: Vertical distance between modules (defaults to stick_length/2)
            rotation_angles: Dict with 'x', 'y', 'z' keys for rotation increments per level in degrees
                            If None, uses default values
            offset_pattern: Pattern for lateral offsets - "spiral", "wave", "random", "fibonacci", or "none"
        
        Returns:
            List of lists, where each inner list contains Stick objects for one module
        """
        from compas.geometry import Rotation, Translation, Frame
        import math
        
        if vertical_spacing is None:
            vertical_spacing = self.stick_length / 2
        
        if rotation_angles is None:
            rotation_angles = {'x': 0, 'y': 0, 'z': 0}
        
        all_modules = []
        
        for i in range(num_modules):
            # Create new frame for this module
            new_frame = self.root_frame.copy()
            
            # Vertical offset
            z_offset = i * vertical_spacing
            new_frame.point += Vector(0, 0, 1) * z_offset
            
            # For odd modules, we need to:
            # 1. Offset in X-Y plane so vertical sticks align between horizontal sticks
            # 2. Then rotate 90 degrees
            if i % 2 == 1:
                # The two x-axis sticks of even modules are at y = -depth and y = +depth
                # We want the vertical stick of odd module (which will be at x = 0 after rotation)
                # to end up at y = 0 (centered between -depth and +depth)
                # Since rotation happens around the frame point, we don't need extra offset
                
                # Just rotate the frame 90 degrees around z-axis
                R_interlock = Rotation.from_axis_and_angle(Vector(0, 0, 1), math.radians(90), point=new_frame.point)
                new_frame.transform(R_interlock)
            
            # Lateral offset based on pattern (applied in world coordinates)
            if offset_pattern == "spiral":
                radius = i * (self.depth * 2)
                angle = i * math.radians(45)
                x_offset = radius * math.cos(angle)
                y_offset = radius * math.sin(angle)
                new_frame.point += Vector(1, 0, 0) * x_offset + Vector(0, 1, 0) * y_offset
                
            elif offset_pattern == "wave":
                amplitude = self.stick_length / 4
                frequency = 0.5
                x_offset = amplitude * math.sin(i * frequency)
                y_offset = amplitude * math.cos(i * frequency)
                new_frame.point += Vector(1, 0, 0) * x_offset + Vector(0, 1, 0) * y_offset
                
            elif offset_pattern == "fibonacci":
                phi = (1 + math.sqrt(5)) / 2
                angle = i * math.radians(137.5)
                radius = i * self.depth * phi
                x_offset = radius * math.cos(angle)
                y_offset = radius * math.sin(angle)
                new_frame.point += Vector(1, 0, 0) * x_offset + Vector(0, 1, 0) * y_offset
                
            elif offset_pattern == "random":
                import random
                random.seed(i)
                x_offset = random.uniform(-self.stick_length/3, self.stick_length/3)
                y_offset = random.uniform(-self.stick_length/3, self.stick_length/3)
                new_frame.point += Vector(1, 0, 0) * x_offset + Vector(0, 1, 0) * y_offset
            
            # Apply additional cumulative rotations if specified
            if rotation_angles['x'] != 0 or rotation_angles['y'] != 0 or rotation_angles['z'] != 0:
                cumulative_x = i * math.radians(rotation_angles['x'])
                cumulative_y = i * math.radians(rotation_angles['y'])
                cumulative_z = i * math.radians(rotation_angles['z'])
                
                center = self.root_frame.point
                
                if cumulative_x != 0:
                    R_x = Rotation.from_axis_and_angle(Vector(1, 0, 0), cumulative_x, point=center)
                    new_frame.transform(R_x)
                
                if cumulative_y != 0:
                    R_y = Rotation.from_axis_and_angle(Vector(0, 1, 0), cumulative_y, point=center)
                    new_frame.transform(R_y)
                
                if cumulative_z != 0:
                    R_z = Rotation.from_axis_and_angle(Vector(0, 0, 1), cumulative_z, point=center)
                    new_frame.transform(R_z)
            
            # Create module using FRAME-RELATIVE coordinates only
            module_sticks = []
            
            # First stick along frame's x-axis
            start_point_1 = (new_frame.point 
                            - new_frame.yaxis * self.depth 
                            - new_frame.xaxis * (self.width/2) 
                            - new_frame.xaxis * (self.stick_length/4))
            stick_axis_1 = Line.from_point_and_vector(start_point_1, new_frame.xaxis * self.stick_length)
            stick_1 = Stick(stick_axis_1, width=self.width, depth=self.depth)
            module_sticks.append(stick_1)
            
            # Second stick moved along y-axis by depth amount
            start_point_2 = (new_frame.point 
                            + new_frame.yaxis * self.depth 
                            - new_frame.xaxis * (self.width/2) 
                            - new_frame.xaxis * (self.stick_length/4))
            stick_axis_2 = Line.from_point_and_vector(start_point_2, new_frame.xaxis * self.stick_length)
            stick_2 = Stick(stick_axis_2, width=self.width, depth=self.depth)
            module_sticks.append(stick_2)
            
            # Third stick along z-axis (this is the one that should be centered!)
            start_point_3 = new_frame.point - new_frame.zaxis * (self.stick_length/4)
            stick_axis_3 = Line.from_point_and_vector(start_point_3, new_frame.zaxis * self.stick_length)
            stick_3 = Stick(stick_axis_3, width=self.width, depth=self.depth)
            module_sticks.append(stick_3)
            
            # Fourth stick along y-axis
            start_point_4 = (new_frame.point 
                            - new_frame.xaxis * (self.depth/2) 
                            - new_frame.yaxis * (self.stick_length/4) 
                            + new_frame.zaxis * self.width 
                            - new_frame.xaxis * (self.width/2) 
                            - new_frame.yaxis * (self.depth/2))
            stick_axis_4 = Line.from_point_and_vector(start_point_4, new_frame.yaxis * self.stick_length)
            stick_4 = Stick(stick_axis_4, width=self.width, depth=self.depth)
            module_sticks.append(stick_4)
            
            all_modules.append(module_sticks)
        
        self.stacked_modules = all_modules
        return all_modules

    def visualize(self):
        """Returns all stick geometries."""
        return [stick.geometry for stick in self.sticks]

    def visualize_stacked(self):
        """
        Returns all geometries from stacked modules.
        
        Returns:
            List of all Box geometries from stacked modules
        """
        if not hasattr(self, 'stacked_modules'):
            return []
        
        geometries = []
        for module_sticks in self.stacked_modules:
            geometries.extend([stick.geometry for stick in module_sticks])
        
        return geometries