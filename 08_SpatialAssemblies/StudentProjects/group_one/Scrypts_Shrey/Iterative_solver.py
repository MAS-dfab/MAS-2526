"""
Stick assembly classes with optimized bridge solver.
Uses geometric constraint satisfaction for fast bridge stick generation.
"""

from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation, Point
from compas.geometry import intersection_line_plane, Scale, intersection_line_line
from compas.geometry import angle_vectors, distance_point_point, closest_point_on_line
import math
import random

# Import from your Sticks_s module
from Sticks_s import stick_from_face_frame


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


from compas.geometry import (
    Point, Vector, Frame, Line, Plane,
    Rotation, distance_point_point, closest_point_on_line
)
import math


# ============================================================================
# CONFIGURATION
# ============================================================================

class BridgeConfig:
    """Configuration parameters for bridge solver."""
    
    DEFAULT_WIDTH = 13.0
    DEFAULT_DEPTH = 13.0
    DEFAULT_LENGTH = 100.0
    
    MAX_BRIDGES = 4
    CONVERGENCE_THRESHOLD = 20.0  # mm
    MIN_ANCHOR = 0.08
    MAX_ANCHOR = 0.92
    MIN_ATTACHMENT_POSITION = 0.15
    MAX_ATTACHMENT_POSITION = 0.85
    
    COLLISION_CLEARANCE = 14.0  # mm
    COLLISION_SLIDE_STEP = 0.05
    MAX_COLLISION_ITERATIONS = 20


# ============================================================================
# GEOMETRIC HELPERS
# ============================================================================

def _vector_from_points(p1, p2):
    """Create vector from p1 to p2."""
    return Vector(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)


def _project_vector_onto_plane(vector, plane_normal):
    """Project vector onto plane defined by normal."""
    normal = plane_normal.unitized()
    dot = vector.x * normal.x + vector.y * normal.y + vector.z * normal.z
    return Vector(
        vector.x - normal.x * dot,
        vector.y - normal.y * dot,
        vector.z - normal.z * dot
    )


def _signed_angle_between_vectors(v1, v2, reference_normal):
    """Calculate signed angle from v1 to v2, with sign determined by reference normal."""
    v1_len = math.sqrt(v1.x**2 + v1.y**2 + v1.z**2)
    v2_len = math.sqrt(v2.x**2 + v2.y**2 + v2.z**2)
    
    if v1_len < 0.001 or v2_len < 0.001:
        return 0.0
    
    v1_unit = Vector(v1.x/v1_len, v1.y/v1_len, v1.z/v1_len)
    v2_unit = Vector(v2.x/v2_len, v2.y/v2_len, v2.z/v2_len)
    
    dot = v1_unit.x * v2_unit.x + v1_unit.y * v2_unit.y + v1_unit.z * v2_unit.z
    cos_angle = max(-1, min(1, dot))
    angle = math.acos(cos_angle)
    
    # Cross product for sign
    cross = Vector(
        v1_unit.y * v2_unit.z - v1_unit.z * v2_unit.y,
        v1_unit.z * v2_unit.x - v1_unit.x * v2_unit.z,
        v1_unit.x * v2_unit.y - v1_unit.y * v2_unit.x
    )
    
    sign_dot = cross.x * reference_normal.x + cross.y * reference_normal.y + cross.z * reference_normal.z
    if sign_dot < 0:
        angle = -angle
    
    return angle


# ============================================================================
# FACE SELECTION
# ============================================================================

def _select_best_face_for_direction(stick, target_direction, attachment_position=0.5):
    """
    Select face that best enables bridge to point toward target_direction.
    
    For lap joints, bridge extends along face.yaxis.
    We want face.zaxis perpendicular to target (so we can rotate to reach it).
    """
    target_len = math.sqrt(target_direction.x**2 + target_direction.y**2 + target_direction.z**2)
    if target_len < 0.001:
        return 0, stick.get_face_frame_at(0, attachment_position), 0
    
    target_unit = Vector(
        target_direction.x / target_len,
        target_direction.y / target_len,
        target_direction.z / target_len
    )
    
    best_face = 0
    best_frame = None
    best_score = -float('inf')
    
    for face_idx in range(4):  # Side faces only
        face_frame = stick.get_face_frame_at(face_idx, attachment_position)
        normal = face_frame.zaxis
        
        # How perpendicular is normal to target? (want ~0)
        dot = abs(normal.x * target_unit.x + normal.y * target_unit.y + normal.z * target_unit.z)
        perpendicularity = 1.0 - dot
        
        # Does face point toward target?
        toward = normal.x * target_unit.x + normal.y * target_unit.y + normal.z * target_unit.z
        toward_bonus = max(0, toward)
        
        # Check if we can actually rotate toward target
        projection = _project_vector_onto_plane(target_direction, normal)
        proj_len = math.sqrt(projection.x**2 + projection.y**2 + projection.z**2)
        can_rotate = proj_len > 0.1
        
        score = perpendicularity * 100 + toward_bonus * 20
        if not can_rotate:
            score -= 50
        
        if score > best_score:
            best_score = score
            best_face = face_idx
            best_frame = face_frame
    
    return best_face, best_frame, best_score


def _select_face_for_vertical(stick, z_direction, attachment_position=0.5):
    """Select face for vertical movement."""
    vertical = Vector(0, 0, z_direction)
    face_idx, frame, _ = _select_best_face_for_direction(stick, vertical, attachment_position)
    return face_idx, frame


# ============================================================================
# ROTATION CALCULATION
# ============================================================================

def _calculate_rotation_to_target(face_frame, target_direction):
    """
    Calculate rotation around face normal to point bridge toward target.
    Bridge extends along face.yaxis after creation.
    """
    normal = face_frame.zaxis
    
    # Project target onto face plane
    projection = _project_vector_onto_plane(target_direction, normal)
    proj_len = math.sqrt(projection.x**2 + projection.y**2 + projection.z**2)
    
    if proj_len < 0.001:
        return 0.0
    
    projection = Vector(projection.x/proj_len, projection.y/proj_len, projection.z/proj_len)
    
    # Current bridge direction is face.yaxis
    current = face_frame.yaxis
    
    return _signed_angle_between_vectors(current, projection, normal)


def _calculate_rotation_for_orientation_match(face_frame, target_axis):
    """Calculate rotation to align bridge parallel to target axis."""
    normal = face_frame.zaxis
    
    projection = _project_vector_onto_plane(target_axis, normal)
    proj_len = math.sqrt(projection.x**2 + projection.y**2 + projection.z**2)
    
    if proj_len < 0.001:
        return 0.0
    
    projection = Vector(projection.x/proj_len, projection.y/proj_len, projection.z/proj_len)
    current = face_frame.yaxis
    
    return _signed_angle_between_vectors(current, projection, normal)


# ============================================================================
# ANCHOR CALCULATION
# ============================================================================

def _calculate_anchor_for_reach(face_frame, rotation_angle, target_point, bridge_length):
    """Calculate anchor position to maximize reach toward target."""
    # Apply rotation
    R = Rotation.from_axis_and_angle(face_frame.zaxis, rotation_angle, face_frame.point)
    rotated_frame = face_frame.copy()
    rotated_frame.transform(R)
    
    bridge_direction = rotated_frame.yaxis
    face_point = face_frame.point
    
    to_target = _vector_from_points(face_point, target_point)
    distance = math.sqrt(to_target.x**2 + to_target.y**2 + to_target.z**2)
    
    if distance < 0.001:
        return 0.5
    
    # How much along bridge direction?
    along = to_target.x * bridge_direction.x + to_target.y * bridge_direction.y + to_target.z * bridge_direction.z
    
    # Desired forward extension
    desired_forward = along * 0.5
    desired_forward = max(-bridge_length * 0.4, min(bridge_length * 0.4, desired_forward))
    
    anchor = 0.5 - (desired_forward / bridge_length)
    
    return max(BridgeConfig.MIN_ANCHOR, min(BridgeConfig.MAX_ANCHOR, anchor))


def _calculate_anchor_for_z_reach(face_frame, rotation_angle, z_target, current_z, bridge_length):
    """Calculate anchor for vertical reach."""
    R = Rotation.from_axis_and_angle(face_frame.zaxis, rotation_angle, face_frame.point)
    rotated_frame = face_frame.copy()
    rotated_frame.transform(R)
    
    bridge_direction = rotated_frame.yaxis
    z_component = bridge_direction.z
    
    if abs(z_component) < 0.001:
        return 0.5
    
    z_diff = z_target - current_z
    z_to_cover = z_diff * 0.5
    forward_needed = z_to_cover / z_component
    forward_needed = max(-bridge_length * 0.4, min(bridge_length * 0.4, forward_needed))
    
    anchor = 0.5 - (forward_needed / bridge_length)
    
    return max(BridgeConfig.MIN_ANCHOR, min(BridgeConfig.MAX_ANCHOR, anchor))


# ============================================================================
# BRIDGE CREATION
# ============================================================================

def _create_bridge(parent_stick, face_idx, attachment_position, rotation_angle, 
                   anchor, bridge_length, width, depth):
    """Create a bridge with given parameters."""
    face_frame = parent_stick.get_face_frame_at(face_idx, attachment_position)
    
    R = Rotation.from_axis_and_angle(face_frame.zaxis, rotation_angle, face_frame.point)
    rotated_face = face_frame.copy()
    rotated_face.transform(R)
    
    return stick_from_face_frame(
        rotated_face, "side", bridge_length,
        width=width, depth=depth,
        anchor_position=anchor
    )


# ============================================================================
# COLLISION HANDLING
# ============================================================================

def _check_collision_clearance(bridge, target_stick, clearance=14.0):
    """
    Check if bridge has clearance from target stick.
    Uses segment distance between axes.
    """
    # Simple segment distance calculation
    p1 = bridge.axis.start
    q1 = bridge.axis.end
    p2 = target_stick.axis.start
    q2 = target_stick.axis.end
    
    def dot(a, b):
        return a.x*b.x + a.y*b.y + a.z*b.z
    
    def sub(a, b):
        return Point(a.x - b.x, a.y - b.y, a.z - b.z)
    
    def add_vec(a, v, s):
        return Point(a.x + v.x*s, a.y + v.y*s, a.z + v.z*s)
    
    u = Vector(q1.x - p1.x, q1.y - p1.y, q1.z - p1.z)
    v = Vector(q2.x - p2.x, q2.y - p2.y, q2.z - p2.z)
    w = Vector(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z)
    
    a = dot(u, u)
    b = dot(u, v)
    c = dot(v, v)
    d = dot(u, w)
    e = dot(v, w)
    
    D = a * c - b * b
    EPS = 1e-9
    
    if D < EPS:
        sN, sD = 0.0, 1.0
        tN, tD = e, c
    else:
        sN = b * e - c * d
        tN = a * e - b * d
        sD = tD = D
        
        if sN < 0.0:
            sN = 0.0
            tN, tD = e, c
        elif sN > sD:
            sN = sD
            tN, tD = e + b, c
    
    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN, sD = -d, a
    elif tN > tD:
        tN = tD
        if -d + b < 0.0:
            sN = 0.0
        elif -d + b > a:
            sN = sD
        else:
            sN, sD = -d + b, a
    
    sc = 0.0 if abs(sN) < EPS else sN / sD
    tc = 0.0 if abs(tN) < EPS else tN / tD
    
    closest1 = add_vec(p1, u, sc)
    closest2 = add_vec(p2, v, tc)
    
    dx = closest1.x - closest2.x
    dy = closest1.y - closest2.y
    dz = closest1.z - closest2.z
    
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    return {
        'has_clearance': distance >= clearance,
        'distance': distance,
        'deficit': max(0, clearance - distance)
    }


def _find_safe_attachment(parent_stick, face_idx, target_stick, bridge_length,
                          rotation_angle, anchor, width, depth, clearance=14.0):
    """Find attachment position avoiding collision."""
    center = 0.5
    positions = [center]
    
    for offset in range(1, 8):
        step = offset * 0.05
        positions.append(min(BridgeConfig.MAX_ATTACHMENT_POSITION, center + step))
        positions.append(max(BridgeConfig.MIN_ATTACHMENT_POSITION, center - step))
    
    # Remove duplicates and sort
    positions = sorted(set(positions))
    
    for pos in positions:
        test_bridge = _create_bridge(
            parent_stick, face_idx, pos, rotation_angle,
            anchor, bridge_length, width, depth
        )
        
        result = _check_collision_clearance(test_bridge, target_stick, clearance)
        
        if result['has_clearance']:
            return pos, anchor, True, result['distance']
    
    return center, anchor, False, 0


# ============================================================================
# MAIN ITERATIVE SOLVER
# ============================================================================

def bridge_sticks_iterative(stick_0, stick_1, bridge_length, width=None, depth=None,
                            max_bridges=None, collision_check=True, debug=False):
    """
    Create bridges using iterative geometric approach.
    
    Builds bridges one at a time, recalculating from actual current geometry.
    
    Parameters:
        stick_0: Starting stick (stick_from_frame or stick_from_face_frame)
        stick_1: Target stick
        bridge_length: Fixed length for bridges
        width: Width (default 13.0)
        depth: Depth (default 13.0)
        max_bridges: Maximum bridges to create (default 4)
        collision_check: Enable collision avoidance (default True)
        debug: Print debug info (default False)
    
    Returns:
        tuple: ([bridges], info_dict)
    """
    width = width or BridgeConfig.DEFAULT_WIDTH
    depth = depth or BridgeConfig.DEFAULT_DEPTH
    max_bridges = max_bridges or BridgeConfig.MAX_BRIDGES
    
    bridges = []
    info = {
        'method': 'iterative_geometric',
        'steps': [],
        'converged': False,
        'final_distance': float('inf'),
        'num_bridges': 0,
        'bridge_length': bridge_length,
        'sequence': []
    }
    
    current_stick = stick_0
    target_point = stick_1.center_frame.point
    target_axis = stick_1.center_frame.xaxis
    
    # Initial distances
    start_point = stick_0.center_frame.point
    initial_distance = distance_point_point(start_point, target_point)
    
    info['initial_distance'] = initial_distance
    info['z_height_diff'] = abs(target_point.z - start_point.z)
    
    if debug:
        print("=" * 60)
        print("ITERATIVE BRIDGE SOLVER")
        print("=" * 60)
        print(f"Start: {start_point}")
        print(f"Target: {target_point}")
        print(f"Initial distance: {initial_distance:.1f}mm")
    
    for i in range(max_bridges):
        step_info = {'bridge_num': i + 1}
        
        # Current state
        current_end = Point(*current_stick.axis.end)
        direction_to_target = _vector_from_points(current_end, target_point)
        distance_to_target = math.sqrt(
            direction_to_target.x**2 + 
            direction_to_target.y**2 + 
            direction_to_target.z**2
        )
        
        step_info['start_distance'] = distance_to_target
        
        if debug:
            print(f"\n--- Bridge {i+1} ---")
            print(f"Current end: {current_end}")
            print(f"Distance to target: {distance_to_target:.1f}mm")
        
        # Check convergence
        if distance_to_target < BridgeConfig.CONVERGENCE_THRESHOLD:
            info['converged'] = True
            info['final_distance'] = distance_to_target
            if debug:
                print(f"CONVERGED! Distance {distance_to_target:.1f}mm < threshold")
            break
        
        # Decompose direction
        xy_direction = Vector(direction_to_target.x, direction_to_target.y, 0)
        xy_distance = math.sqrt(xy_direction.x**2 + xy_direction.y**2)
        z_diff = direction_to_target.z
        
        if debug:
            print(f"  XY distance: {xy_distance:.1f}mm, Z diff: {z_diff:.1f}mm")
        
        # Strategy selection
        if i == 0 and xy_distance > bridge_length * 0.3:
            # First bridge: horizontal movement
            strategy = 'xy_movement'
            attachment_pos = 0.8
            
            face_idx, face_frame, _ = _select_best_face_for_direction(
                current_stick, xy_direction, attachment_pos
            )
            rotation_angle = _calculate_rotation_to_target(face_frame, xy_direction)
            anchor = _calculate_anchor_for_reach(
                face_frame, rotation_angle, target_point, bridge_length
            )
            
        elif abs(z_diff) > bridge_length * 0.2:
            # Vertical movement needed
            strategy = 'z_movement'
            attachment_pos = 0.7
            z_direction = 1 if z_diff > 0 else -1
            
            face_idx, face_frame = _select_face_for_vertical(
                current_stick, z_direction, attachment_pos
            )
            
            vertical = Vector(0, 0, z_direction)
            rotation_angle = _calculate_rotation_to_target(face_frame, vertical)
            
            current_z = current_end.z
            anchor = _calculate_anchor_for_z_reach(
                face_frame, rotation_angle, target_point.z, current_z, bridge_length
            )
            
        else:
            # Direct approach
            strategy = 'direct_approach'
            attachment_pos = 0.75
            
            face_idx, face_frame, _ = _select_best_face_for_direction(
                current_stick, direction_to_target, attachment_pos
            )
            
            # Try orientation matching if close
            if distance_to_target < bridge_length * 0.8:
                rotation_angle = _calculate_rotation_for_orientation_match(
                    face_frame, target_axis
                )
                strategy = 'orientation_match'
            else:
                rotation_angle = _calculate_rotation_to_target(
                    face_frame, direction_to_target
                )
            
            anchor = _calculate_anchor_for_reach(
                face_frame, rotation_angle, target_point, bridge_length
            )
        
        step_info['strategy'] = strategy
        step_info['face_idx'] = face_idx
        step_info['rotation_deg'] = math.degrees(rotation_angle)
        step_info['anchor'] = anchor
        
        if debug:
            print(f"  Strategy: {strategy}")
            print(f"  Face: {face_idx}, Rotation: {math.degrees(rotation_angle):.1f}°, Anchor: {anchor:.2f}")
        
        # Collision handling
        collision_resolved = True
        if collision_check:
            safe_pos, safe_anchor, success, clearance_dist = _find_safe_attachment(
                current_stick, face_idx, stick_1, bridge_length,
                rotation_angle, anchor, width, depth
            )
            
            if not success:
                collision_resolved = False
                if debug:
                    print(f"  WARNING: Could not resolve collision")
            elif safe_pos != attachment_pos:
                attachment_pos = safe_pos
                anchor = safe_anchor
                if debug:
                    print(f"  Collision avoided: new position={safe_pos:.2f}")
            
            step_info['collision_resolved'] = collision_resolved
            step_info['clearance'] = clearance_dist
        
        # Create bridge
        bridge = _create_bridge(
            current_stick, face_idx, attachment_pos, rotation_angle,
            anchor, bridge_length, width, depth
        )
        
        bridges.append(bridge)
        info['steps'].append(step_info)
        info['sequence'].append((
            chr(65 + i),  # 'A', 'B', 'C', ...
            attachment_pos,
            face_idx,
            anchor,
            math.degrees(rotation_angle)
        ))
        
        # Store face frame for visualization
        if i == 0:
            info['best_face'] = face_idx
            info['face_frame_A'] = face_frame
        
        current_stick = bridge
        
        if debug:
            new_end = bridge.axis.end
            new_dist = distance_point_point(Point(*new_end), target_point)
            print(f"  New distance: {new_dist:.1f}mm")
    
    # Final state
    final_end = Point(*current_stick.axis.end)
    info['final_distance'] = distance_point_point(final_end, target_point)
    info['num_bridges'] = len(bridges)
    
    if debug:
        print("\n" + "=" * 60)
        print(f"RESULT: {len(bridges)} bridges")
        print(f"Final distance: {info['final_distance']:.1f}mm")
        print(f"Converged: {info['converged']}")
        print("=" * 60)
    
    return bridges, info