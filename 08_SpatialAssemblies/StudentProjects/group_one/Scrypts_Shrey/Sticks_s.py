"""
Stick assembly classes with optimized bridge solver.
Uses geometric constraint satisfaction for fast bridge stick generation.
"""

from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation, Point
from compas.geometry import intersection_line_plane, Scale, intersection_line_line
from compas.geometry import angle_vectors, distance_point_point, closest_point_on_line
import math
import random


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def _calculate_z_vector_from_centerline(centerline_vector):
    """Calculate perpendicular z-vector from a centerline direction."""
    c = Vector(0, 0, 1)
    angle = angle_vectors(c, centerline_vector)
    if angle < 0.001 or angle > math.pi - 0.001:
        c = Vector(1, 0, 0)
    return c


# ============================================================================
# ORIGINAL STICK CLASS (Legacy)
# ============================================================================

class Stick:
    """Legacy stick class - uses axis-based definition."""
    
    SIZE = 10.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, axis, z_vector=None, width=None, depth=None):
        self.axis = axis
        self.z_vector = z_vector
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self._get_stick_frame()
        self.length = axis.length  # Add length property
    
    def _get_stick_frame(self):
        normal = _calculate_z_vector_from_centerline(self.axis.direction)
        xaxis = self.axis.direction.unitized()
        yaxis = normal.unitized()
        frame = Frame(self.axis.midpoint, xaxis, yaxis)
        return frame

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth, self.frame)
        return box
    
    def get_face_frame(self, face_idx):
        """
        Get frame at a face of the stick.
        Face indexing: 0-3 are the four side faces around the stick.
        """
        # Get center frame
        if not hasattr(self, 'center_frame'):
            add_center_frame_to_stick(self)
        
        center = self.center_frame.point
        
        # Face offsets (perpendicular to stick axis)
        if face_idx == 0:
            offset = self.center_frame.yaxis * (self.width / 2)
        elif face_idx == 1:
            offset = -self.center_frame.yaxis * (self.width / 2)
        elif face_idx == 2:
            offset = self.center_frame.zaxis * (self.depth / 2)
        elif face_idx == 3:
            offset = -self.center_frame.zaxis * (self.depth / 2)
        else:
            raise ValueError(f"Face index must be 0-3, got {face_idx}")
        
        face_point = center + offset
        
        # Face frame: xaxis along stick, zaxis = outward normal
        if face_idx == 0:
            face_frame = Frame(face_point, self.center_frame.xaxis, self.center_frame.yaxis)
        elif face_idx == 1:
            face_frame = Frame(face_point, self.center_frame.xaxis, -self.center_frame.yaxis)
        elif face_idx == 2:
            face_frame = Frame(face_point, self.center_frame.xaxis, self.center_frame.zaxis)
        else:  # face_idx == 3
            face_frame = Frame(face_point, self.center_frame.xaxis, -self.center_frame.zaxis)
        
        return face_frame
    
    def get_face_frame_at(self, face_idx, t):
        """
        Get frame at a face at position t along the stick (0 to 1).
        
        Args:
            face_idx: Face index (0-3)
            t: Position along stick (0.0 = start, 1.0 = end)
        """
        # Get center frame
        if not hasattr(self, 'center_frame'):
            add_center_frame_to_stick(self)
        
        # Calculate point at position t along the stick
        point_on_axis = self.axis.start + (self.axis.end - self.axis.start) * t
        
        # Face offsets
        if face_idx == 0:
            offset = self.center_frame.yaxis * (self.width / 2)
            normal = self.center_frame.yaxis
        elif face_idx == 1:
            offset = -self.center_frame.yaxis * (self.width / 2)
            normal = -self.center_frame.yaxis
        elif face_idx == 2:
            offset = self.center_frame.zaxis * (self.depth / 2)
            normal = self.center_frame.zaxis
        elif face_idx == 3:
            offset = -self.center_frame.zaxis * (self.depth / 2)
            normal = -self.center_frame.zaxis
        else:
            raise ValueError(f"Face index must be 0-3, got {face_idx}")
        
        face_point = point_on_axis + offset
        
        # Create frame: xaxis along stick, zaxis = outward normal
        face_frame = Frame(face_point, self.center_frame.xaxis, normal)
        
        return face_frame
    
    def rotate_stick(self, angle, rotation_axis=None, pt=None):
        if not rotation_axis:
            rotation_axis = self.axis.direction
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(angle), pt or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)
        # Update center_frame if it exists
        if hasattr(self, 'center_frame'):
            self.center_frame.transform(R)
    
    def rotate_stick_random(self, min_angle=0, max_angle=360, rotation_axis=None, pt=None, seed=None):
        """Rotate the stick around its frame normal by a random angle."""
        if seed is not None:
            random.seed(seed)
            seed_used = seed
        else:
            seed_used = random.randint(0, 1000000)
            random.seed(seed_used)
        
        random_angle = random.uniform(min_angle, max_angle)
        if not rotation_axis:
            rotation_axis = self.frame.normal 
        R = Rotation.from_axis_and_angle(rotation_axis, math.radians(random_angle), pt or self.axis.midpoint)
        self.frame.transform(R)
        self.axis.transform(R)
        # Update center_frame if it exists
        if hasattr(self, 'center_frame'):
            self.center_frame.transform(R)
        
        return random_angle, seed_used


# Add this function OUTSIDE the Stick class
def add_center_frame_to_stick(old_stick):
    """Add center_frame attribute to old-style Stick objects"""
    if not hasattr(old_stick, 'center_frame'):
        # Calculate center point
        center_point = Point(
            (old_stick.axis.start.x + old_stick.axis.end.x) / 2,
            (old_stick.axis.start.y + old_stick.axis.end.y) / 2,
            (old_stick.axis.start.z + old_stick.axis.end.z) / 2
        )
        
        # Get stick direction (xaxis)
        stick_direction = Vector.from_start_end(old_stick.axis.start, old_stick.axis.end).unitized()
        
        # Create frame (using existing frame's yaxis and zaxis if available)
        if hasattr(old_stick, 'frame'):
            center_frame = Frame(center_point, stick_direction, old_stick.frame.yaxis)
        else:
            # Create default frame
            if abs(stick_direction.dot(Vector(0, 0, 1))) < 0.9:
                zaxis = Vector(0, 0, 1)
            else:
                zaxis = Vector(1, 0, 0)
            yaxis = zaxis.cross(stick_direction).unitized()
            zaxis = stick_direction.cross(yaxis).unitized()
            center_frame = Frame(center_point, stick_direction, yaxis)
        
        old_stick.center_frame = center_frame
    
    return old_stick
# ============================================================================
# LEGACY BRIDGING FUNCTIONS (Simple geometry-based)
# ============================================================================

def stick_bridge(stick0, stick1):
    """Legacy simple bridge function."""
    plane0 = Plane(stick0.axis.midpoint, stick0.frame.xaxis)
    pt1 = intersection_line_plane(stick1.axis, plane0)
    
    plane1 = Plane(stick1.axis.midpoint, stick1.frame.xaxis)
    pt0 = intersection_line_plane(stick0.axis, plane1)
    
    return Stick(Line(pt0, stick1.axis.midpoint)), Stick(Line(pt1, stick0.axis.midpoint)), Stick(Line(pt0, pt1)), pt0, pt1, plane0, plane1


def stick_bridge_endpoints(stick0, stick1):
    """Bridge two sticks by connecting their endpoints."""
    stick0_start = stick0.axis.start
    stick0_end = stick0.axis.end
    stick1_start = stick1.axis.start
    stick1_end = stick1.axis.end
    
    bridge_start_start = Stick(Line(stick0_start, stick1_start))
    bridge_end_end = Stick(Line(stick0_end, stick1_end))
    
    return bridge_start_start, bridge_end_end


def stick_bridge_axis_aligned_overlap(stick0, stick1, connection_point0=None, connection_point1=None, overlap_length=None):
    """
    Bridge two sticks using axis-aligned sticks with overlapping joints.
    Creates a path of sticks aligned to X, Y, Z axes with lateral offsets.
    """
    if connection_point0 is None:
        connection_point0 = stick0.axis.midpoint
    if connection_point1 is None:
        connection_point1 = stick1.axis.midpoint
    
    if overlap_length is None:
        overlap_length = stick0.depth
    
    delta = connection_point1 - connection_point0
    dx, dy, dz = delta.x, delta.y, delta.z
    
    bridge_sticks = []
    current_point = connection_point0.copy()
    
    # Get stick directions
    stick0_dir = stick0.axis.direction.unitized()
    stick1_dir = stick1.axis.direction.unitized()
    
    # Order movements by magnitude
    movements = [
        ('x', abs(dx), dx, Vector(1, 0, 0)),
        ('y', abs(dy), dy, Vector(0, 1, 0)),
        ('z', abs(dz), dz, Vector(0, 0, 1))
    ]
    movements.sort(key=lambda m: m[1], reverse=True)
    movements = [(n, m, s, v) for n, m, s, v in movements if m > 0.001]
    
    if len(movements) == 0:
        return []
    
    # Reorder to avoid parallel connections
    if len(movements) >= 2:
        import itertools
        best_movements = None
        best_score = -1
        parallel_threshold = 0.9
        
        for perm in itertools.permutations(movements):
            perm_list = list(perm)
            first_dir = perm_list[0][3] * (1 if perm_list[0][2] > 0 else -1)
            last_dir = perm_list[-1][3] * (1 if perm_list[-1][2] > 0 else -1)
            
            dot_first_check = abs(stick0_dir.dot(first_dir))
            dot_last_check = abs(stick1_dir.dot(last_dir))
            
            score = 0
            if dot_first_check < parallel_threshold:
                score += 100
            if dot_last_check < parallel_threshold:
                score += 100
            score -= dot_first_check * 10
            score -= dot_last_check * 10
            
            if score > best_score:
                best_score = score
                best_movements = perm_list
        
        if best_movements:
            movements = best_movements
    
    # Calculate initial offset
    first_bridge_dir = movements[0][3] * (1 if movements[0][2] > 0 else -1)
    cross = stick0_dir.cross(first_bridge_dir)
    
    if cross.length > 0.001:
        cumulative_lateral_offset = cross.unitized() * stick0.width
    else:
        cumulative_lateral_offset = Vector(0, 0, 0)
    
    for i, (axis_name, magnitude, signed_dist, axis_vector) in enumerate(movements):
        direction = axis_vector * (1 if signed_dist > 0 else -1)
        
        # Calculate perpendicular offset for subsequent sticks
        if i > 0:
            prev_axis_name = movements[i-1][0]
            
            if axis_name == 'x':
                offset_increment = Vector(0, 0, stick0.width) if prev_axis_name == 'y' else Vector(0, stick0.width, 0)
            elif axis_name == 'y':
                offset_increment = Vector(0, 0, stick0.width) if prev_axis_name == 'x' else Vector(stick0.width, 0, 0)
            else:  # z
                offset_increment = Vector(0, stick0.width, 0) if prev_axis_name == 'x' else Vector(stick0.width, 0, 0)
            
            cumulative_lateral_offset += offset_increment
        
        # Calculate start and end points
        if i == 0:
            start_pt = current_point + cumulative_lateral_offset - direction * overlap_length
        else:
            start_pt = current_point + cumulative_lateral_offset - direction * overlap_length
        
        if i == len(movements) - 1:
            end_pt = connection_point1 + cumulative_lateral_offset + direction * overlap_length
        else:
            end_pt = current_point + direction * magnitude + cumulative_lateral_offset + direction * overlap_length
        
        # Create bridge stick
        bridge_axis = Line(start_pt, end_pt)
        z_vec = _calculate_z_vector_from_centerline(direction)
        
        bridge_stick = Stick(bridge_axis, z_vector=z_vec, width=stick0.width, depth=stick0.depth)
        bridge_sticks.append(bridge_stick)
        
        current_point = current_point + direction * magnitude
    
    return bridge_sticks


# ============================================================================
# FRAME-BASED STICK CLASSES
# ============================================================================

class stick_from_frame:
    """A stick defined by a center frame."""
    
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, center_frame, stick_length, width=None, depth=None):
        """
        Create a stick using a center frame.
        
        Parameters:
            center_frame: Frame at the center (xaxis = stick direction)
            stick_length: Length of the stick
            width: Width (defaults to SIZE)
            depth: Depth (defaults to SIZE)
        """
        self.center_frame = center_frame
        self.length = stick_length
        self.width = width or stick_from_frame.WIDTH
        self.depth = depth or stick_from_frame.DEPTH
        
        half_length = self.length / 2
        self.axis = Line(
            center_frame.point - center_frame.xaxis * half_length,
            center_frame.point + center_frame.xaxis * half_length
        )
    
    @property
    def start_point(self):
        return self.axis.start
    
    @property
    def end_point(self):
        return self.axis.end
    
    @property
    def direction(self):
        return self.center_frame.xaxis
    
    @property
    def geometry(self):
        return Box(self.length, self.width, self.depth, self.center_frame)
    
    @property
    def frame(self):
        return self.center_frame
    
    @property
    def face_frames(self):
        """Returns frames for all 6 faces (zaxis points outward)."""
        frames = []
        center = self.center_frame.point
        
        # Face 0: +Y (top)
        face0_point = center + self.center_frame.yaxis * (self.width / 2)
        face0_frame = Frame(face0_point, self.center_frame.xaxis, -self.center_frame.zaxis)
        frames.append(face0_frame)
        
        # Face 1: -Y (bottom)
        face1_point = center - self.center_frame.yaxis * (self.width / 2)
        face1_frame = Frame(face1_point, self.center_frame.xaxis, self.center_frame.zaxis)
        frames.append(face1_frame)
        
        # Face 2: +Z (right)
        face2_point = center + self.center_frame.zaxis * (self.depth / 2)
        face2_frame = Frame(face2_point, self.center_frame.xaxis, self.center_frame.yaxis)
        frames.append(face2_frame)
        
        # Face 3: -Z (left)
        face3_point = center - self.center_frame.zaxis * (self.depth / 2)
        face3_frame = Frame(face3_point, self.center_frame.xaxis, -self.center_frame.yaxis)
        frames.append(face3_frame)
        
        # Face 4: +X (end)
        face4_point = center + self.center_frame.xaxis * (self.length / 2)
        face4_frame = Frame(face4_point, self.center_frame.yaxis, self.center_frame.zaxis)
        frames.append(face4_frame)
        
        # Face 5: -X (start)
        face5_point = center - self.center_frame.xaxis * (self.length / 2)
        face5_frame = Frame(face5_point, self.center_frame.yaxis, -self.center_frame.zaxis)
        frames.append(face5_frame)
        
        return frames
    
    def get_face_frame(self, face_index):
        """Get frame of specific face (0-5)."""
        if 0 <= face_index < 6:
            return self.face_frames[face_index]
        else:
            raise ValueError(f"Face index must be 0-5, got {face_index}")
    
    def get_face_frame_at(self, face_index, position=0.5):
        """Get frame at position along face (0.0=start, 0.5=middle, 1.0=end)."""
        if not (0 <= face_index < 6):
            raise ValueError(f"Face index must be 0-5, got {face_index}")
        
        # End faces don't vary with position
        if face_index in [4, 5]:
            return self.face_frames[face_index]
        
        # Side faces offset along stick axis
        base_frame = self.face_frames[face_index]
        offset_factor = position - 0.5
        offset_distance = offset_factor * self.length
        new_point = base_frame.point + self.center_frame.xaxis * offset_distance
        
        return Frame(new_point, base_frame.xaxis, base_frame.yaxis)
    
    def __repr__(self):
        return f"stick_from_frame(center={self.center_frame.point}, length={self.length})"


class stick_from_face_frame:
    """A stick defined by a face frame (zaxis points outward)."""
    
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, face_frame, face_type, stick_length, width=None, depth=None, anchor_position=0.5):
        """
        Create a stick from a face frame.
        
        Parameters:
            face_frame: Frame on face (xaxis, yaxis on surface, zaxis outward)
            face_type: "side" (faces 0-3) or "end" (faces 4-5)
            stick_length: Length of the stick
            width: Width (defaults to SIZE)
            depth: Depth (defaults to SIZE)
            anchor_position: Where to anchor (0.0=start, 0.5=middle, 1.0=end)
        """
        self.width = width or stick_from_face_frame.WIDTH
        self.depth = depth or stick_from_face_frame.DEPTH
        self.length = stick_length
        self.face_frame = face_frame
        self.face_type = face_type
        self.anchor_position = anchor_position
        
        self.center_frame = self._calculate_center_frame()
        
        half_length = self.length / 2
        self.axis = Line(
            self.center_frame.point - self.center_frame.xaxis * half_length,
            self.center_frame.point + self.center_frame.xaxis * half_length
        )
    
    def _calculate_center_frame(self):
        """Calculate center frame from face frame with anchor offset."""
        face_pt = self.face_frame.point
        anchor_offset = (0.5 - self.anchor_position) * self.length
        
        if self.face_type == "side" or self.face_type in [0, 1, 2, 3]:
            stick_xaxis = self.face_frame.yaxis
            center_pt = face_pt + self.face_frame.zaxis * (self.width / 2) + stick_xaxis * anchor_offset
            stick_yaxis = self.face_frame.zaxis
            return Frame(center_pt, stick_xaxis, stick_yaxis)
        else:  # end face
            stick_xaxis = self.face_frame.zaxis
            center_pt = face_pt + self.face_frame.zaxis * (self.length / 2 + anchor_offset)
            stick_yaxis = self.face_frame.yaxis
            return Frame(center_pt, stick_xaxis, stick_yaxis)
    
    @property
    def start_point(self):
        return self.axis.start
    
    @property
    def end_point(self):
        return self.axis.end
    
    @property
    def direction(self):
        return self.center_frame.xaxis
    
    @property
    def geometry(self):
        return Box(self.length, self.width, self.depth, self.center_frame)
    
    @property
    def frame(self):
        return self.center_frame
    
    @property
    def face_frames(self):
        """Returns frames for all 6 faces."""
        frames = []
        center = self.center_frame.point
        
        face0_point = center + self.center_frame.yaxis * (self.width / 2)
        frames.append(Frame(face0_point, self.center_frame.xaxis, -self.center_frame.zaxis))
        
        face1_point = center - self.center_frame.yaxis * (self.width / 2)
        frames.append(Frame(face1_point, self.center_frame.xaxis, self.center_frame.zaxis))
        
        face2_point = center + self.center_frame.zaxis * (self.depth / 2)
        frames.append(Frame(face2_point, self.center_frame.xaxis, self.center_frame.yaxis))
        
        face3_point = center - self.center_frame.zaxis * (self.depth / 2)
        frames.append(Frame(face3_point, self.center_frame.xaxis, -self.center_frame.yaxis))
        
        face4_point = center + self.center_frame.xaxis * (self.length / 2)
        frames.append(Frame(face4_point, self.center_frame.yaxis, self.center_frame.zaxis))
        
        face5_point = center - self.center_frame.xaxis * (self.length / 2)
        frames.append(Frame(face5_point, self.center_frame.yaxis, -self.center_frame.zaxis))
        
        return frames
    
    def get_face_frame(self, face_index):
        if 0 <= face_index < 6:
            return self.face_frames[face_index]
        else:
            raise ValueError(f"Face index must be 0-5, got {face_index}")
    
    def get_face_frame_at(self, face_index, position=0.5):
        if not (0 <= face_index < 6):
            raise ValueError(f"Face index must be 0-5, got {face_index}")
        
        if face_index in [4, 5]:
            return self.face_frames[face_index]
        
        base_frame = self.face_frames[face_index]
        offset_factor = position - 0.5
        offset_distance = offset_factor * self.length
        new_point = base_frame.point + self.center_frame.xaxis * offset_distance
        
        return Frame(new_point, base_frame.xaxis, base_frame.yaxis)
    
    def __repr__(self):
        return f"stick_from_face_frame(center={self.center_frame.point}, length={self.length})"




def generate_all_frame_configurations(stick_0, stick_1):
    """
    Generate all 64 possible frame configurations for two sticks.
    
    Each stick has 8 configurations:
    - X-axis: original direction OR flipped 180° (2 options)
    - Z-axis: rotated 0°, 90°, 180°, 270° around X-axis (4 options)
    Total per stick: 2 × 4 = 8 configurations
    Total combinations: 8 × 8 = 64
    
    Returns:
        list: List of 64 dictionaries, each containing:
            - config_id: 1-64
            - stick_0_x_flip: 0 (original) or 1 (flipped 180°)
            - stick_0_z_rot: 0, 90, 180, or 270 degrees
            - stick_1_x_flip: 0 or 1
            - stick_1_z_rot: 0, 90, 180, or 270 degrees
            - frame_0: transformed frame for stick_0
            - frame_1: transformed frame for stick_1
            - z_angle_rad, y_angle_rad, x_angle_rad: ZYX Euler angles in radians
            - z_angle_deg, y_angle_deg, x_angle_deg: ZYX Euler angles in degrees
            - angle_sum: sum of absolute angles
            - max_angle: maximum absolute angle
            - description: text description
    """
    from compas.geometry import Rotation
    import math
    
    configs = []
    config_id = 1
    
    # Get original frames
    original_frame_0 = stick_0.center_frame.copy()
    original_frame_1 = stick_1.center_frame.copy()
    
    def get_frame_variations(original_frame):
        """
        Get 8 frame variations for a stick.
        Returns list of (frame, x_flip, z_rot_deg, description)
        """
        variations = []
        
        # For each X-axis direction (original and flipped)
        for x_flip in [0, 1]:
            # Start with original frame
            base_frame = original_frame.copy()
            
            if x_flip == 1:
                # Flip X-axis: rotate 180° around Z-axis
                axis = base_frame.zaxis.scaled(math.pi)  # axis-angle vector
                point = [base_frame.point.x, base_frame.point.y, base_frame.point.z]
                R_flip = Rotation.from_axis_angle_vector(axis, point)
                base_frame.transform(R_flip)
            
            # For each Z-axis rotation around X-axis
            for z_rot_deg in [0, 90, 180, 270]:
                frame_rotated = base_frame.copy()
                
                if z_rot_deg > 0:
                    # Rotate around X-axis
                    z_rot_rad = math.radians(z_rot_deg)
                    axis = base_frame.xaxis.scaled(z_rot_rad)  # axis-angle vector
                    point = [base_frame.point.x, base_frame.point.y, base_frame.point.z]
                    R_rot = Rotation.from_axis_angle_vector(axis, point)
                    frame_rotated.transform(R_rot)
                
                x_desc = "X-flip" if x_flip == 1 else "X-orig"
                desc = f"{x_desc}, Z-rot{z_rot_deg}"
                
                variations.append((frame_rotated, x_flip, z_rot_deg, desc))
        
        return variations
    
    # Get variations for both sticks
    stick_0_variations = get_frame_variations(original_frame_0)
    stick_1_variations = get_frame_variations(original_frame_1)
    
    # Generate all 64 combinations (8 × 8)
    for frame_0, x_flip_0, z_rot_0, desc_0 in stick_0_variations:
        for frame_1, x_flip_1, z_rot_1, desc_1 in stick_1_variations:
            
            # Extract ZYX Euler angles for this configuration
            z_angle, y_angle, x_angle = extract_zyx_euler_angles(frame_0, frame_1)
            
            config = {
                'config_id': config_id,
                'stick_0_x_flip': x_flip_0,
                'stick_0_z_rot': z_rot_0,
                'stick_1_x_flip': x_flip_1,
                'stick_1_z_rot': z_rot_1,
                'frame_0': frame_0.copy(),
                'frame_1': frame_1.copy(),
                'z_angle_rad': z_angle,
                'y_angle_rad': y_angle,
                'x_angle_rad': x_angle,
                'z_angle_deg': math.degrees(z_angle),
                'y_angle_deg': math.degrees(y_angle),
                'x_angle_deg': math.degrees(x_angle),
                'description': f"S0[{desc_0}] + S1[{desc_1}]",
                'angle_sum': abs(z_angle) + abs(y_angle) + abs(x_angle),
                'max_angle': max(abs(z_angle), abs(y_angle), abs(x_angle))
            }
            
            configs.append(config)
            config_id += 1
    
    return configs

def extract_zyx_euler_angles(frame_0, frame_1):
    """
    Extract ZYX Euler angles between two frames.
    
    Parameters:
        frame_0: Reference frame
        frame_1: Target frame
    
    Returns:
        tuple: (z_angle, y_angle, x_angle) in radians
    """
    import math
    from compas.geometry import Transformation
    
    # Compute relative transformation
    T_0 = Transformation.from_frame(frame_0)
    T_1 = Transformation.from_frame(frame_1)
    T_rel = T_1 * T_0.inverted()
    
    # Get rotation matrix
    R = [[T_rel.matrix[i][j] for j in range(3)] for i in range(3)]
    
    # Extract ZYX Euler angles
    try:
        y_angle = math.asin(max(-1, min(1, -R[2][0])))
        
        if abs(math.cos(y_angle)) > 0.00001:
            z_angle = math.atan2(R[1][0], R[0][0])
            x_angle = math.atan2(R[2][1], R[2][2])
        else:
            # Gimbal lock case
            z_angle = math.atan2(-R[0][1], R[1][1])
            x_angle = 0
        
        return z_angle, y_angle, x_angle
    except:
        return 0, 0, 0


def bridge_sticks_euler_config(stick_0, stick_1, bridge_length, config_id, 
                                width=None, depth=None, angle_tolerance=0.01, 
                                bridge_A_start=0.5, print_info=True,
                                return_separate=False):
    """
    Create bridges using a specific configuration ID (1-64).
    
    Uses ZYX Euler sequence only.
    
    Parameters:
        stick_0: First stick
        stick_1: Second stick
        bridge_length: Length of bridge sticks
        config_id: Configuration ID (1-64)
        width: Bridge width (default: 13.0)
        depth: Bridge depth (default: 13.0)
        angle_tolerance: Angle tolerance for bridge creation (default: 0.01 rad)
        bridge_A_start: Where Bridge A starts on stick_0 (default: 0.5)
        print_info: Whether to print debug info (default: True)
        return_separate: If True, return (bridge_A, bridge_B, bridge_C, info_dict) 
                        instead of (bridges, info_dict) (default: False)
    
    Returns:
        If return_separate=False (default):
            tuple: (bridges, info_dict)
        If return_separate=True:
            tuple: (bridge_A, bridge_B, bridge_C, info_dict)
            where bridge_A, bridge_B, bridge_C are individual bridges or None
    """
    import math
    from compas.geometry import Rotation, Plane
    
    if width is None:
        width = 13.0
    if depth is None:
        depth = 13.0
    
    # Generate all 64 configurations
    all_configs = generate_all_frame_configurations(stick_0, stick_1)
    
    # Validate config_id
    if config_id < 1 or config_id > len(all_configs):
        raise ValueError(f"config_id must be between 1 and {len(all_configs)}, got {config_id}")
    
    # Get selected configuration
    config = all_configs[config_id - 1]
    
    if print_info:
        print("=" * 60)
        print(f"USING CONFIGURATION {config_id}/64")
        print("=" * 60)
        print(f"Stick_0: X-flip={config['stick_0_x_flip']}, Z-rot={config['stick_0_z_rot']}°")
        print(f"Stick_1: X-flip={config['stick_1_x_flip']}, Z-rot={config['stick_1_z_rot']}°")
        print(f"Euler ZYX: Z={config['z_angle_deg']:.1f}°, Y={config['y_angle_deg']:.1f}°, X={config['x_angle_deg']:.1f}°")
        print(f"Description: {config['description']}")
        print("=" * 60)
    
    # Use the configured frames
    frame_0 = config['frame_0']
    frame_1 = config['frame_1']
    
    z_angle = config['z_angle_rad']
    y_angle = config['y_angle_rad']
    x_angle = config['x_angle_rad']
    
    # Calculate distances
    z_diff = abs(frame_1.point.z - frame_0.point.z)
    stick_1_xy_projection = Point(frame_1.point.x, frame_1.point.y, frame_0.point.z)
    xy_distance = distance_point_point(
        Point(frame_0.point.x, frame_0.point.y, frame_0.point.z), 
        stick_1_xy_projection
    )
    
    bridges = []
    bridge_sequence = []
    
    # Individual bridge tracking
    bridge_A = None
    bridge_B = None
    bridge_C = None
    
    # Create temporary sticks with the configured frames
    temp_stick_0 = stick_from_frame(frame_0, stick_0.length, width=stick_0.width, depth=stick_0.depth)
    temp_stick_1 = stick_from_frame(frame_1, stick_1.length, width=stick_1.width, depth=stick_1.depth)
    
    current_stick = temp_stick_0
    best_face_idx = 0
    
    info = {
        'method': 'Euler_config',
        'config_id': config_id,
        'stick_0_x_flip': config['stick_0_x_flip'],
        'stick_0_z_rot': config['stick_0_z_rot'],
        'stick_1_x_flip': config['stick_1_x_flip'],
        'stick_1_z_rot': config['stick_1_z_rot'],
        'euler_sequence': 'ZYX',
        'z_angle_deg': math.degrees(z_angle),
        'y_angle_deg': math.degrees(y_angle),
        'x_angle_deg': math.degrees(x_angle),
        'z_height_diff': z_diff,
        'xy_distance': xy_distance,
        'bridge_length': bridge_length,
        'bridge_A_start': bridge_A_start,
        'description': config['description']
    }
    
    # Find best horizontal face
    world_z = Vector(0, 0, 1)
    best_alignment = 0
    for face_idx in range(4):
        face_frame = temp_stick_0.get_face_frame(face_idx)
        alignment = abs(face_frame.zaxis.dot(world_z))
        if alignment > best_alignment:
            best_alignment = alignment
            best_face_idx = face_idx
    
    # ============================================================================
    # Bridge A - Horizontal movement (Z-angle rotation)
    # ============================================================================
    if abs(z_angle) > angle_tolerance or xy_distance > 1.0:
        face_frame = temp_stick_0.get_face_frame_at(best_face_idx, bridge_A_start)
        
        R = Rotation.from_axis_and_angle(face_frame.zaxis, z_angle, face_frame.point)
        rotated_face = face_frame.copy()
        rotated_face.transform(R)
        
        bridge_A_temp = stick_from_face_frame(rotated_face, "side", bridge_length, width, depth, anchor_position=0.5)
        x_slide = xy_distance / 2.0
        
        if x_slide > bridge_length:
            anchor_A = 0.1
        else:
            dist_start = distance_point_point(bridge_A_temp.axis.start, stick_1_xy_projection)
            dist_end = distance_point_point(bridge_A_temp.axis.end, stick_1_xy_projection)
            anchor_A = 0.5 - (x_slide / bridge_length) if dist_end < dist_start else 0.5 + (x_slide / bridge_length)
            anchor_A = max(0.0, min(1.0, anchor_A))
        
        bridge_A = stick_from_face_frame(rotated_face, "side", bridge_length, width, depth, anchor_position=anchor_A)
        bridges.append(bridge_A)
        bridge_sequence.append(('A', z_angle, anchor_A))
        current_stick = bridge_A
    
    # ============================================================================
    # Bridge B - Vertical/Roll (X-angle rotation)
    # ============================================================================
    if abs(x_angle) > angle_tolerance or z_diff > 1.0:
        bridge_A_axis = current_stick.axis
        bridge_A_vector_full = Vector.from_start_end(bridge_A_axis.start, bridge_A_axis.end)
        bridge_A_direction = bridge_A_vector_full.unitized()
        
        stick_1_direction = Vector.from_start_end(temp_stick_1.axis.start, temp_stick_1.axis.end).unitized()
        
        plane_normal = bridge_A_direction.copy()
        R_roll = Rotation.from_axis_and_angle(stick_1_direction, x_angle, temp_stick_1.center_frame.point)
        rotated_plane_normal = plane_normal.copy()
        rotated_plane_normal.transform(R_roll)
        
        roll_plane = Plane(temp_stick_1.center_frame.point, rotated_plane_normal)
        info['roll_plane'] = roll_plane
        
        intersection_pt = intersection_line_plane(bridge_A_axis, roll_plane)
        
        if intersection_pt:
            if isinstance(intersection_pt, list):
                intersection_pt = Point(*intersection_pt)
            info['intersection_point'] = intersection_pt
            to_intersection = Vector.from_start_end(bridge_A_axis.start, intersection_pt)
            t = to_intersection.dot(bridge_A_vector_full) / (bridge_A_vector_full.length ** 2) if bridge_A_vector_full.length > 0.001 else 0.5
            attachment_position = max(0.1, min(0.9, t))
        else:
            attachment_position = 0.5
        
        stick_1_midpoint = temp_stick_1.axis.midpoint
        adjacent_face_1 = (best_face_idx + 1) % 4
        adjacent_face_2 = best_face_idx % 4
        
        face_frame_B1 = current_stick.get_face_frame_at(adjacent_face_1, attachment_position)
        face_frame_B2 = current_stick.get_face_frame_at(adjacent_face_2, attachment_position)
        
        dist_1 = distance_point_point(face_frame_B1.point, stick_1_midpoint)
        dist_2 = distance_point_point(face_frame_B2.point, stick_1_midpoint)
        
        next_face_idx = adjacent_face_1 if dist_1 < dist_2 else adjacent_face_2
        face_frame_B = face_frame_B1 if dist_1 < dist_2 else face_frame_B2
        
        if abs(x_angle) > angle_tolerance:
            rotation_angle = x_angle if next_face_idx in [0, 2] else -x_angle
            R_B = Rotation.from_axis_and_angle(face_frame_B.zaxis, rotation_angle, face_frame_B.point)
            rotated_face_B = face_frame_B.copy()
            rotated_face_B.transform(R_B)
            face_frame_B = rotated_face_B
        
        # Collision check
        temp_bridge_B = stick_from_face_frame(face_frame_B, "side", bridge_length, width, depth)
        cp_B = closest_point_on_line(stick_1_midpoint, temp_bridge_B.axis)
        cp_1 = closest_point_on_line(cp_B, temp_stick_1.axis)
        intersection_distance = distance_point_point(cp_B, cp_1)
        
        if intersection_distance < depth:
            slide_distance_base = depth - intersection_distance
            bridge_B_direction = Vector.from_start_end(temp_bridge_B.axis.start, temp_bridge_B.axis.end).unitized()
            stick_1_axis_direction = Vector.from_start_end(temp_stick_1.axis.start, temp_stick_1.axis.end).unitized()
            
            cos_angle = abs(bridge_B_direction.dot(stick_1_axis_direction))
            sin_angle = max(math.sqrt(1 - cos_angle**2), 0.1)
            slide_distance = slide_distance_base / sin_angle
            
            bridge_A_direction_slide = current_stick.center_frame.xaxis
            direction_to_stick1 = Vector.from_start_end(face_frame_B.point, stick_1_midpoint).unitized()
            slide_amount = slide_distance / current_stick.length
            
            attachment_position_adjusted = attachment_position - slide_amount if direction_to_stick1.dot(bridge_A_direction_slide) > 0 else attachment_position + slide_amount
            attachment_position_adjusted_clamped = max(0.1, min(0.9, attachment_position_adjusted))
            
            if abs(attachment_position_adjusted - attachment_position_adjusted_clamped) > 0.01:
                attachment_position_adjusted = attachment_position - slide_amount if attachment_position_adjusted > attachment_position else attachment_position + slide_amount
                attachment_position_adjusted_clamped = max(0.1, min(0.9, attachment_position_adjusted))
            
            attachment_position = attachment_position_adjusted_clamped
            face_frame_B = current_stick.get_face_frame_at(next_face_idx, attachment_position)
            
            if abs(x_angle) > angle_tolerance:
                rotation_angle = x_angle if next_face_idx in [0, 2] else -x_angle
                R_B = Rotation.from_axis_and_angle(face_frame_B.zaxis, rotation_angle, face_frame_B.point)
                rotated_face_B = face_frame_B.copy()
                rotated_face_B.transform(R_B)
                face_frame_B = rotated_face_B
        
        # Calculate anchor
        z_extension_needed = z_diff / 2.0
        temp_bridge_for_direction = stick_from_face_frame(face_frame_B, "side", bridge_length, width, depth, anchor_position=0.5)
        points_upward = temp_bridge_for_direction.center_frame.xaxis.dot(Vector(0, 0, 1)) > 0
        stick_1_above = frame_1.point.z > frame_0.point.z
        
        if z_extension_needed > bridge_length:
            anchor_B = 0.1
        else:
            if (stick_1_above and points_upward) or (not stick_1_above and not points_upward):
                anchor_B = 0.5 - (z_extension_needed / bridge_length)
            else:
                anchor_B = 0.5 + (z_extension_needed / bridge_length)
            anchor_B = max(0.1, min(0.9, anchor_B))
        
        bridge_B = stick_from_face_frame(face_frame_B, "side", bridge_length, width, depth, anchor_position=anchor_B)
        bridges.append(bridge_B)
        bridge_sequence.append(('B', attachment_position, next_face_idx, anchor_B))
        current_stick = bridge_B
    
    # ============================================================================
    # Bridge C - Pitch (Y-angle rotation)
    # ============================================================================
    if len(bridges) > 0:
        bridge_A_axis_ref = bridges[0].axis
        bridge_A_start_xy = Point(bridge_A_axis_ref.start.x, bridge_A_axis_ref.start.y, frame_0.point.z)
        bridge_A_end_xy = Point(bridge_A_axis_ref.end.x, bridge_A_axis_ref.end.y, frame_0.point.z)
        bridge_A_axis_xy = Line(bridge_A_start_xy, bridge_A_end_xy)
        
        stick_1_center_xy = Point(frame_1.point.x, frame_1.point.y, frame_0.point.z)
        closest_on_bridge_A_xy = closest_point_on_line(stick_1_center_xy, bridge_A_axis_xy)
        
        bridge_A_vec = Vector.from_start_end(bridge_A_start_xy, bridge_A_end_xy)
        to_closest_vec = Vector.from_start_end(bridge_A_start_xy, closest_on_bridge_A_xy)
        
        t_xy = to_closest_vec.dot(bridge_A_vec) / (bridge_A_vec.length ** 2) if bridge_A_vec.length > 0.001 else 0.5
        intersects_bridge_A = (0.0 <= t_xy <= 1.0)
    else:
        intersects_bridge_A = True
    
    if abs(y_angle) > angle_tolerance or not intersects_bridge_A:
        bridge_C_ref_axis = current_stick.axis
        bridge_C_ref_vector_full = Vector.from_start_end(bridge_C_ref_axis.start, bridge_C_ref_axis.end)
        bridge_C_ref_direction = bridge_C_ref_vector_full.unitized()
        
        stick_1_direction = Vector.from_start_end(temp_stick_1.axis.start, temp_stick_1.axis.end).unitized()
        
        plane_normal_C = bridge_C_ref_direction.copy()
        R_pitch = Rotation.from_axis_and_angle(stick_1_direction, y_angle, temp_stick_1.center_frame.point)
        rotated_plane_normal_C = plane_normal_C.copy()
        rotated_plane_normal_C.transform(R_pitch)
        
        pitch_plane = Plane(temp_stick_1.center_frame.point, rotated_plane_normal_C)
        info['pitch_plane'] = pitch_plane
        
        intersection_pt_C = intersection_line_plane(bridge_C_ref_axis, pitch_plane)
        
        if intersection_pt_C:
            if isinstance(intersection_pt_C, list):
                intersection_pt_C = Point(*intersection_pt_C)
            
            to_intersection_C = Vector.from_start_end(bridge_C_ref_axis.start, intersection_pt_C)
            t_C = to_intersection_C.dot(bridge_C_ref_vector_full) / (bridge_C_ref_vector_full.length ** 2) if bridge_C_ref_vector_full.length > 0.001 else 0.5
            attachment_position_C = max(0.1, min(0.9, t_C))
        else:
            attachment_position_C = 0.5
        
        # Find best face
        best_face_C = 0
        max_alignment = 0
        face_frame_C = None
        
        for face_idx in range(4):
            face_frame_temp = current_stick.get_face_frame_at(face_idx, attachment_position_C)
            alignment = abs(face_frame_temp.zaxis.dot(stick_1_direction))
            
            if alignment > max_alignment:
                max_alignment = alignment
                best_face_C = face_idx
                face_frame_C = face_frame_temp
        
        next_face_idx_C = best_face_C
        
        if abs(y_angle) > angle_tolerance:
            rotation_angle_C = y_angle if next_face_idx_C in [0, 2] else -y_angle
            R_C = Rotation.from_axis_and_angle(face_frame_C.zaxis, rotation_angle_C, face_frame_C.point)
            rotated_face_C = face_frame_C.copy()
            rotated_face_C.transform(R_C)
            face_frame_C = rotated_face_C
        
        # Collision check
        temp_bridge_C = stick_from_face_frame(face_frame_C, "side", bridge_length, width, depth)
        cp_C = closest_point_on_line(temp_stick_1.axis.midpoint, temp_bridge_C.axis)
        cp_1_C = closest_point_on_line(cp_C, temp_stick_1.axis)
        intersection_distance_C = distance_point_point(cp_C, cp_1_C)
        
        if intersection_distance_C < depth:
            slide_distance_base_C = depth - intersection_distance_C
            bridge_C_direction = Vector.from_start_end(temp_bridge_C.axis.start, temp_bridge_C.axis.end).unitized()
            stick_1_axis_direction = Vector.from_start_end(temp_stick_1.axis.start, temp_stick_1.axis.end).unitized()
            
            cos_angle_C = abs(bridge_C_direction.dot(stick_1_axis_direction))
            sin_angle_C = max(math.sqrt(1 - cos_angle_C**2), 0.1)
            slide_distance_C = slide_distance_base_C / sin_angle_C
            
            bridge_C_ref_direction_slide = current_stick.center_frame.xaxis
            direction_to_stick1_C = Vector.from_start_end(face_frame_C.point, temp_stick_1.axis.midpoint).unitized()
            slide_amount_C = slide_distance_C / current_stick.length
            
            attachment_position_C_adjusted = attachment_position_C - slide_amount_C if direction_to_stick1_C.dot(bridge_C_ref_direction_slide) > 0 else attachment_position_C + slide_amount_C
            attachment_position_C_adjusted_clamped = max(0.1, min(0.9, attachment_position_C_adjusted))
            
            if abs(attachment_position_C_adjusted - attachment_position_C_adjusted_clamped) > 0.01:
                attachment_position_C_adjusted = attachment_position_C - slide_amount_C if attachment_position_C_adjusted > attachment_position_C else attachment_position_C + slide_amount_C
                attachment_position_C_adjusted_clamped = max(0.1, min(0.9, attachment_position_C_adjusted))
            
            attachment_position_C = attachment_position_C_adjusted_clamped
            face_frame_C = current_stick.get_face_frame_at(next_face_idx_C, attachment_position_C)
            
            if abs(y_angle) > angle_tolerance:
                rotation_angle_C = y_angle if next_face_idx_C in [0, 2] else -y_angle
                R_C = Rotation.from_axis_and_angle(face_frame_C.zaxis, rotation_angle_C, face_frame_C.point)
                rotated_face_C = face_frame_C.copy()
                rotated_face_C.transform(R_C)
                face_frame_C = rotated_face_C
        
        # Calculate anchor
        remaining_distance = distance_point_point(face_frame_C.point, temp_stick_1.center_frame.point)
        anchor_extension_C = remaining_distance / 2.0
        
        temp_bridge_C_dir = stick_from_face_frame(face_frame_C, "side", bridge_length, width, depth, anchor_position=0.5)
        bridge_C_dir = temp_bridge_C_dir.center_frame.xaxis
        direction_to_stick1_final = Vector.from_start_end(face_frame_C.point, temp_stick_1.center_frame.point).unitized()
        points_toward = bridge_C_dir.dot(direction_to_stick1_final) > 0
        
        if anchor_extension_C > bridge_length:
            anchor_C = 0.1
        else:
            anchor_C = 0.5 - (anchor_extension_C / bridge_length) if points_toward else 0.5 + (anchor_extension_C / bridge_length)
            anchor_C = max(0.1, min(0.9, anchor_C))
        
        bridge_C = stick_from_face_frame(face_frame_C, "side", bridge_length, width, depth, anchor_position=anchor_C)
        bridges.append(bridge_C)
        bridge_sequence.append(('C', attachment_position_C, next_face_idx_C, anchor_C))
    
    info['num_bridges'] = len(bridges)
    info['bridge_sequence'] = bridge_sequence
    info['best_face'] = best_face_idx
    
    # Return based on return_separate flag
    if return_separate:
        return bridge_A, bridge_B, bridge_C, info
    else:
        return bridges, info


def find_best_configuration(stick_0, stick_1, criterion='min_angles'):
    """
    Find the best configuration out of 64 options.
    
    Parameters:
        stick_0: First stick
        stick_1: Second stick
        criterion: Selection criterion
            - 'min_angles': Minimize sum of absolute angles (default)
            - 'min_max': Minimize the largest angle
            - 'min_z': Minimize Z rotation
            - 'min_y': Minimize Y rotation
            - 'min_x': Minimize X rotation
    
    Returns:
        int: Best config_id (1-64)
    """
    all_configs = generate_all_frame_configurations(stick_0, stick_1)
    
    if criterion == 'min_angles':
        best = min(all_configs, key=lambda c: c['angle_sum'])
    elif criterion == 'min_max':
        best = min(all_configs, key=lambda c: c['max_angle'])
    elif criterion == 'min_z':
        best = min(all_configs, key=lambda c: abs(c['z_angle_rad']))
    elif criterion == 'min_y':
        best = min(all_configs, key=lambda c: abs(c['y_angle_rad']))
    elif criterion == 'min_x':
        best = min(all_configs, key=lambda c: abs(c['x_angle_rad']))
    else:
        best = all_configs[0]
    
    return best['config_id']


def print_all_configurations(stick_0, stick_1, top_n=10):
    """
    Print top N configurations sorted by angle sum.
    
    Parameters:
        stick_0: First stick
        stick_1: Second stick
        top_n: Number of top configurations to print (default: 10)
    """
    import math
    
    all_configs = generate_all_frame_configurations(stick_0, stick_1)
    sorted_configs = sorted(all_configs, key=lambda c: c['angle_sum'])
    
    print("=" * 80)
    print(f"TOP {top_n} CONFIGURATIONS (out of 64)")
    print("=" * 80)
    
    for i, config in enumerate(sorted_configs[:top_n]):
        print(f"\nRank {i+1}: Config ID {config['config_id']}")
        print(f"  S0: X-flip={config['stick_0_x_flip']}, Z-rot={config['stick_0_z_rot']}°")
        print(f"  S1: X-flip={config['stick_1_x_flip']}, Z-rot={config['stick_1_z_rot']}°")
        print(f"  ZYX: Z={config['z_angle_deg']:>6.1f}°, Y={config['y_angle_deg']:>6.1f}°, X={config['x_angle_deg']:>6.1f}°")
        print(f"  Sum={math.degrees(config['angle_sum']):.1f}°, Max={math.degrees(config['max_angle']):.1f}°")
    
    print("\n" + "=" * 80)


# ============================================================================
# USAGE EXAMPLES
# ============================================================================

# Example 1: Use specific configuration
# bridges, info = bridge_sticks_euler_config(stick_0, stick_1, 100, config_id=15)

# Example 2: Find and use best configuration
# best_id = find_best_configuration(stick_0, stick_1, criterion='min_angles')
# bridges, info = bridge_sticks_euler_config(stick_0, stick_1, 100, config_id=best_id)

# Example 3: Review all options
# print_all_configurations(stick_0, stick_1, top_n=10)

# Example 4: Test multiple configurations
# for config_id in range(1, 11):
#     bridges, info = bridge_sticks_euler_config(stick_0, stick_1, 100, config_id, print_info=False)
#     print(f"Config {config_id}: {len(bridges)} bridges, sum={info['z_angle_deg']+info['y_angle_deg']+info['x_angle_deg']:.1f}°")