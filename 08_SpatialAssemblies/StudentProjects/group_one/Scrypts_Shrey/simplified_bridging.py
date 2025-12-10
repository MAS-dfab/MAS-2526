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

