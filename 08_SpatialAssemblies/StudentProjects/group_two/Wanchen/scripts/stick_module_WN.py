from compas.geometry import Line, Frame, Vector, Point
from compas.geometry import Rotation
from compas.geometry import distance_point_point
from Sticks import Stick

import math
import random

class LoopBranchModuleStable:
    """
    input is one frame
    generate a closed loop of sticks with alternating up/down offsets
    """

    def __init__(self,
                 root_frame,
                 stick_length=600,
                 num_segments=8,
                 min_turn=math.radians(20),
                 max_turn=math.radians(140),
                 overlap_distance=20.0,
                 last_length_limit=50.0,
                 offset=0,
                 depth=None,
                 width=None,
                 min_distance=30.0,
                 closure_tol=1.0,
                 max_attempts=200):
        self.root_frame = root_frame
        self.stick_length = float(stick_length)
        self.num_segments = int(num_segments)
        self.min_turn = float(min_turn)
        self.max_turn = float(max_turn)
        self.overlap_distance = float(overlap_distance)
        self.last_length_limit = float(last_length_limit)
        self.offset = float(offset)
        self.depth = depth or Stick.DEPTH
        self.width = width or Stick.WIDTH
        self.min_distance = float(min_distance)
        self.closure_tol = float(closure_tol)
        self.max_attempts = int(max_attempts)

        self.points = []   # planar anchor points (before vertical offset)
        self.lines = []    # Line segments used for sticks (with start/end including overlap & vertical offset)
        self.sticks = []   # final Stick objects

        # Try to build a valid loop immediately
        ok = self._generate_valid_loop()
        if ok:
            self._build_sticks_from_lines()
        else:
            # leave empty if failed
            self.points = []
            self.lines = []
            self.sticks = []

    # ----------------------------
    def _random_angle(self):
        """Return random turning angle within bounds (signed)."""
        a = random.uniform(self.min_turn, self.max_turn)
        return a if random.random() < 0.5 else -a

    # ----------------------------
    def _segment_midpoint(self, a: Point, b: Point):
        return Point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)

    # ----------------------------
    def _segments_too_close(self, segments):
        """Lightweight check: ensure midpoints of non-adjacent segments are not too close."""
        mids = [self._segment_midpoint(s[0], s[1]) for s in segments]
        n = len(mids)
        for i in range(n):
            for j in range(i + 2, n):
                # skip adjacent pairs and the pair (first,last) if they are adjacent in chain
                # for circular adjacency, treat last and first as adjacent (so skip j == i-1 mod n)
                if i == 0 and j == n - 1:
                    continue
                if distance_point_point(mids[i], mids[j]) < self.min_distance:
                    return True
        return False

    # ----------------------------
    def _generate_valid_loop(self):
        """
        Attempt up to max_attempts to build a valid loop.
        Returns True if succeeded and fills self.points and self.lines (with overlap and vertical offset).
        """

        root = Point(*self.root_frame.point)
        for attempt in range(self.max_attempts):
            # 1) Generate planar anchor points (before overlap adjustments)
            pts = [root]
            direction = self.root_frame.xaxis  # initial direction (compas Vector)
            prev_angle = 0.0
            for i in range(self.num_segments - 1):
                angle = self._random_angle()
                prev_angle = angle
                # rotate direction around root_frame.zaxis
                direction = direction.rotated(angle, self.root_frame.zaxis)
                # next anchor point (no overlap applied yet)
                nxt = Point(
                    pts[-1].x + direction.x * self.stick_length,
                    pts[-1].y + direction.y * self.stick_length,
                    pts[-1].z + direction.z * self.stick_length,
                )
                pts.append(nxt)

            # Now decide last_start by applying overlap on penultimate segment:
            # default last_start = pts[-1] + unit_vec_to_root * 0 (we will try offsets)
            # We'll sample a few offsets along the previous segment direction to try to meet closure tolerance
            prev_pt = pts[-1]
            prev_dir_vec = Vector.from_start_end(pts[-2], pts[-1])
            if prev_dir_vec.length == 0:
                continue
            prev_dir_unit = Vector(prev_dir_vec.x / prev_dir_vec.length,
                                   prev_dir_vec.y / prev_dir_vec.length,
                                   prev_dir_vec.z / prev_dir_vec.length)

            # sample offsets from -overlap ... +overlap (so last start can move backward/forward)
            # include zero and a few values
            samples = [0.0, -self.overlap_distance, self.overlap_distance,
                       -self.overlap_distance * 0.5, self.overlap_distance * 0.5]
            found = False

            for s in samples:
                # compute last_start applying this sampled offset along previous segment direction
                last_start = Point(prev_pt.x + prev_dir_unit.x * s,
                                   prev_pt.y + prev_dir_unit.y * s,
                                   prev_pt.z + prev_dir_unit.z * s)

                # compute vector from last_start to root
                vec_to_root = Vector.from_start_end(last_start, root)
                dist_to_root = vec_to_root.length
                if dist_to_root == 0:
                    # already at root; degenerate
                    final_length = 0.0
                    final_end = root
                else:
                    # constrain final length within limits
                    max_len = self.stick_length + self.last_length_limit
                    min_len = max(0.0, self.stick_length - self.last_length_limit)
                    final_length = min(dist_to_root, max_len)
                    final_length = max(final_length, min_len)
                    unit_vec = Vector(vec_to_root.x / dist_to_root,
                                      vec_to_root.y / dist_to_root,
                                      vec_to_root.z / dist_to_root)
                    final_end = Point(
                        last_start.x + unit_vec.x * final_length,
                        last_start.y + unit_vec.y * final_length,
                        last_start.z + unit_vec.z * final_length
                    )

                # does final_end reach root within tolerance? if so accept
                if distance_point_point(final_end, root) <= self.closure_tol:
                    # Build the full sequence of anchor points used for axes starts (including adjusted last_start and final_end)
                    anchor_pts = pts[:-1]  # all except penultimate (we will replace the last two)
                    # first N-1 stick: we will treat start points as anchor_pts[i] (but we'll apply overlap when building axes)
                    anchor_pts.append(last_start)   # this becomes the start of final segment
                    anchor_pts.append(final_end)    # the end point
                    # Now compute axes (with overlap applied): for each i, start_i and end_i
                    axes = []
                    for i in range(len(anchor_pts) - 1):
                        a = anchor_pts[i]
                        b = anchor_pts[i + 1]
                        # apply overlap: move start along direction from previous segment end
                        # For first segment start uses anchor a (no further adjustment)
                        # For segment i>0, we want start = previous_end - prev_dir * overlap_distance
                        if i == 0:
                            start_pt = a
                        else:
                            # previous raw direction
                            prev_a = anchor_pts[i - 1]
                            prev_dir = Vector.from_start_end(prev_a, a)
                            plen = prev_dir.length if prev_dir.length != 0 else 1.0
                            prev_unit = Vector(prev_dir.x / plen, prev_dir.y / plen, prev_dir.z / plen)
                            start_pt = Point(a.x - prev_unit.x * self.overlap_distance,
                                             a.y - prev_unit.y * self.overlap_distance,
                                             a.z - prev_unit.z * self.overlap_distance)
                        end_pt = b
                        axes.append((start_pt, end_pt))
                    # quick collision heuristic
                    if self._segments_too_close(axes):
                        continue
                    # success: set self.points and self.lines (with vertical offsets applied later)
                    self.points = [pt for pt in anchor_pts]   # planar anchor points
                    # store axes temporarily as planar starts/ends (we'll add vertical offset in stick-building)
                    self._temp_axes = axes
                    found = True
                    break

            if found:
                # success for this attempt -> lines will be created later by build_sticks
                return True
            # else try another random sequence
        # exhausted attempts
        return False

    # ----------------------------
    def _build_sticks_from_lines(self):
        """Use self._temp_axes (list of (start_pt,end_pt)) to make Stick objects with alternating z offsets."""
        self.lines = []
        self.sticks = []
        half = self.depth / 2.0
        up = True
        for (s_pt, e_pt) in self._temp_axes:
            # vertical offset
            vec = e_pt - s_pt
            dir_unit = vec.unitized()

            start_off = Point(s_pt.x - dir_unit.x * self.offset,
                                s_pt.y - dir_unit.y * self.offset,
                                s_pt.z - dir_unit.z * self.offset)
            end_off = Point(e_pt.x + dir_unit.x * self.offset,
                                e_pt.y + dir_unit.y * self.offset,
                                e_pt.z + dir_unit.z * self.offset)
            zdir = Vector(0, 0, 1) if up else Vector(0, 0, -1)
            offs = Vector(zdir.x * half, zdir.y * half, zdir.z * half)
            start_off = Point(start_off.x + offs.x, start_off.y + offs.y, start_off.z + offs.z)
            end_off = Point(end_off.x + offs.x, end_off.y + offs.y, end_off.z + offs.z)
            line = Line(start_off, end_off)
            self.lines.append(line)
            stick = Stick(line, z_vector=zdir, width=self.width, depth=self.depth)
            self.sticks.append(stick)
            up = not up

    # ----------------------------
    def visualize(self):
        return [s.geometry for s in self.sticks]

class TowerGenerator:
    """
    Generate a vertical tower with multiple loop levels connected by vertical sticks.
    All sticks have face offsets for proper joinery.
    """

    def __init__(self,
                 root_frame,
                 num_levels=3,
                 level_height=400,
                 stick_length=200,
                 num_segments=8,
                 min_turn=math.radians(20),
                 max_turn=math.radians(140),
                 overlap_distance=20.0,
                 last_length_limit=50.0,
                 offset=5.0,
                 depth=13.0,
                 width=13.0,
                 min_distance=30.0,
                 closure_tol=1.0,
                 max_attempts=200,
                 connector_stick_width=13.0,
                 seed=None):
        
        if seed is not None:
            random.seed(seed)
        
        self.root_frame = root_frame
        self.num_levels = int(num_levels)
        self.level_height = float(level_height)
        self.offset = float(offset)
        self.depth = float(depth)
        self.width = float(width)
        self.connector_stick_width = float(connector_stick_width)

        # Store parameters for loop generation
        self.loop_params = {
            'stick_length': stick_length,
            'num_segments': num_segments,
            'min_turn': min_turn,
            'max_turn': max_turn,
            'overlap_distance': overlap_distance,
            'last_length_limit': last_length_limit,
            'offset': offset,
            'depth': depth,
            'width': width,
            'min_distance': min_distance,
            'closure_tol': closure_tol,
            'max_attempts': max_attempts
        }

        self.loop_modules = []
        self.connector_sticks = []
        self.all_sticks = []

        self.build_tower()
    
    def build_tower(self):
        level_frames = []

        # Generate loops at each level
        for level_idx in range(self.num_levels):
            # Create frame for this level
            z_offset = level_idx * self.level_height
            level_frame = self.root_frame.copy()
            level_frame = level_frame.translated(Vector(0, 0, z_offset))
            level_frames.append(level_frame)

            # Generate loop at this level
            loop = LoopBranchModuleStable(level_frame, **self.loop_params)
            if len(loop.sticks) > 0:
                self.loop_modules.append(loop)
                self.all_sticks.extend(loop.sticks)
        
        # Generate connector sticks between levels  
        if len(self.loop_modules) > 1:
            for i in range(len(self.loop_modules) - 1):
                self._connect_levels(i, i + 1)

    def _connect_levels(self, level_a_idx: int, level_b_idx: int):
        """
        Connect two adjacent levels with vertical connector sticks.
        FIXED: Explicitly pass proper z_vector for vertical sticks
        """
        loop_a = self.loop_modules[level_a_idx]
        loop_b = self.loop_modules[level_b_idx]

        if len(loop_a.points) == 0 or len(loop_b.points) == 0:
            return

        pts_a = loop_a.points
        pts_b = loop_b.points

        num_connectors = min(len(pts_a), len(pts_b))

        for idx in range(num_connectors):
            pt_a = pts_a[idx]
            pt_b = pts_b[idx]

            # Create vertical connector with face offset
            vec = Vector.from_start_end(pt_a, pt_b)

            if vec.length < 0.01:
                continue  # Skip degenerate cases

            dir_unit = vec.unitized()

            # Apply offset along direction
            start_off = Point(
                pt_a.x - dir_unit.x * self.offset,
                pt_a.y - dir_unit.y * self.offset,
                pt_a.z - dir_unit.z * self.offset
            )
            end_off = Point(
                pt_b.x + dir_unit.x * self.offset,
                pt_b.y + dir_unit.y * self.offset,
                pt_b.z + dir_unit.z * self.offset
            )

            # Create line for the stick
            line = Line(start_off, end_off)
            
            try:
                # KEY FIX: 显式传递z_vector为XY平面内的向量
                # 因为connector是垂直的，z_vector应该与Z轴垂直
                # 使用水平方向作为z_vector参考
                
                # 获取XY平面的投影方向（消除Z分量）
                xy_direction = Vector(dir_unit.x, dir_unit.y, 0)
                
                if xy_direction.length > 0.01:
                    # 如果有有效的水平分量，使用它作为参考
                    z_vector = xy_direction.unitized()
                else:
                    # 如果connector是完全垂直的（XY投影为零）
                    # 使用X轴作为默认z_vector
                    z_vector = Vector(1, 0, 0)
                
                # 创建stick，显式指定z_vector，避免调用_calculate_z_vector_from_centerline
                stick = Stick(
                    line,
                    z_vector=z_vector,  # 显式传递，不要让它为None
                    width=self.connector_stick_width,
                    depth=self.depth
                )
                self.connector_sticks.append(stick)
                self.all_sticks.append(stick)
                
            except Exception as e:
                print(f"Warning: Failed to create connector stick at index {idx}: {e}")
                continue

    # ----------------------------
    def visualize(self):
        return [s.geometry for s in self.all_sticks]
    
    def get_loop_geometries(self):
        """Return only loop stick geometries."""
        geoms = []
        for loop in self.loop_modules:
            geoms.extend(loop.visualize())
        return geoms
    
    def get_connector_geometries(self):
        """Return only connector stick geometries."""
        return [s.geometry for s in self.connector_sticks]
    
    def export_for_fabrication(self):
        """
        Export stick data optimized for robot gluing.
        Return list of dicts with joinery info.
        """
        fabrication_data = []

        for stick in self.all_sticks:
            data = {
                'geometry': stick.geometry,
                'start_point': stick.line.start,
                'end_point': stick.line.end,
                'width': stick.width,
                'depth': stick.depth,
                'z_vector': stick.z_vector,
                'face_offset': self.offset,
                'type': 'loop' if stick in [s for loop in self.loop_modules for s in loop.sticks] else 'connector'                
            }
            fabrication_data.append(data)

        return fabrication_data
    
# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def distance_point_point(p1: Point, p2: Point) -> float:
    """Calculate distance between two points."""
    return p1.distance_to_point(p2)

class LoopBranchModule3D:
    """
    input is one frame
    generate a closed loop of sticks in 3D space with alternating up/down offsets
    """

    def __init__(self,
                 root_frame,
                 stick_length=600,
                 num_segments=8,
                 min_turn=math.radians(20),
                 max_turn=math.radians(140),
                 overlap_distance=20.0,
                 last_length_limit=50.0,
                 offset=0,
                 depth=None,
                 width=None,
                 min_distance=30.0,
                 closure_tol=1.0,
                 max_attempts=200,
                 rotation_mode='full3d'):  # 'full3d' or 'constrained'
        self.root_frame = root_frame
        self.stick_length = float(stick_length)
        self.num_segments = int(num_segments)
        self.min_turn = float(min_turn)
        self.max_turn = float(max_turn)
        self.overlap_distance = float(overlap_distance)
        self.last_length_limit = float(last_length_limit)
        self.offset = float(offset)
        self.depth = depth or Stick.DEPTH
        self.width = width or Stick.WIDTH
        self.min_distance = float(min_distance)
        self.closure_tol = float(closure_tol)
        self.max_attempts = int(max_attempts)
        self.rotation_mode = rotation_mode

        self.points = []   # 3D anchor points (before vertical offset)
        self.lines = []    # Line segments used for sticks (with start/end including overlap & vertical offset)
        self.sticks = []   # final Stick objects

        # Try to build a valid loop immediately
        ok = self._generate_valid_loop()
        if ok:
            self._build_sticks_from_lines()
        else:
            # leave empty if failed
            self.points = []
            self.lines = []
            self.sticks = []

    # ----------------------------
    def _random_angle(self):
        """Return random turning angle within bounds (signed)."""
        a = random.uniform(self.min_turn, self.max_turn)
        return a if random.random() < 0.5 else -a

    # ----------------------------
    def _random_rotation_axis(self, current_direction):
        """
        Generate a random rotation axis perpendicular to current direction.
        This creates natural 3D turns.
        """
        # Create two perpendicular vectors to current direction
        if abs(current_direction.x) < 0.9:
            temp = Vector(1, 0, 0)
        else:
            temp = Vector(0, 1, 0)
        
        # perpendicular vector 1
        perp1 = current_direction.cross(temp)
        if perp1.length > 0:
            perp1 = perp1.unitized()
        else:
            perp1 = Vector(0, 0, 1)
        
        # perpendicular vector 2
        perp2 = current_direction.cross(perp1)
        if perp2.length > 0:
            perp2 = perp2.unitized()
        
        # Random combination of the two perpendicular vectors
        theta = random.uniform(0, 2 * math.pi)
        axis = Vector(
            perp1.x * math.cos(theta) + perp2.x * math.sin(theta),
            perp1.y * math.cos(theta) + perp2.y * math.sin(theta),
            perp1.z * math.cos(theta) + perp2.z * math.sin(theta)
        )
        
        return axis.unitized() if axis.length > 0 else perp1

    # ----------------------------
    def _segment_midpoint(self, a: Point, b: Point):
        return Point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)

    # ----------------------------
    def _segments_too_close(self, segments):
        """Lightweight check: ensure midpoints of non-adjacent segments are not too close."""
        mids = [self._segment_midpoint(s[0], s[1]) for s in segments]
        n = len(mids)
        for i in range(n):
            for j in range(i + 2, n):
                # skip adjacent pairs and the pair (first,last) if they are adjacent in chain
                if i == 0 and j == n - 1:
                    continue
                if distance_point_point(mids[i], mids[j]) < self.min_distance:
                    return True
        return False

    # ----------------------------
    def _generate_valid_loop(self):
        """
        Attempt up to max_attempts to build a valid 3D loop.
        Returns True if succeeded and fills self.points and self.lines.
        """

        root = Point(*self.root_frame.point)
        for attempt in range(self.max_attempts):
            # 1) Generate 3D anchor points
            pts = [root]
            direction = self.root_frame.xaxis  # initial direction (compas Vector)
            
            for i in range(self.num_segments - 1):
                angle = self._random_angle()
                
                # 3D rotation: choose rotation axis based on mode
                if self.rotation_mode == 'full3d':
                    # Full 3D: rotate around a random axis perpendicular to current direction
                    rotation_axis = self._random_rotation_axis(direction)
                elif self.rotation_mode == 'constrained':
                    # Constrained: mix of Z-axis rotation and perpendicular rotation
                    if random.random() < 0.7:  # 70% Z-axis, 30% perpendicular
                        rotation_axis = self.root_frame.zaxis
                    else:
                        rotation_axis = self._random_rotation_axis(direction)
                else:
                    # Default to Z-axis only (original 2D behavior)
                    rotation_axis = self.root_frame.zaxis
                
                # Rotate direction around the chosen axis
                direction = direction.rotated(angle, rotation_axis)
                
                # Next anchor point
                nxt = Point(
                    pts[-1].x + direction.x * self.stick_length,
                    pts[-1].y + direction.y * self.stick_length,
                    pts[-1].z + direction.z * self.stick_length,
                )
                pts.append(nxt)

            # Close the loop (same logic as before but now in 3D)
            prev_pt = pts[-1]
            prev_dir_vec = Vector.from_start_end(pts[-2], pts[-1])
            if prev_dir_vec.length == 0:
                continue
            prev_dir_unit = Vector(prev_dir_vec.x / prev_dir_vec.length,
                                   prev_dir_vec.y / prev_dir_vec.length,
                                   prev_dir_vec.z / prev_dir_vec.length)

            # Sample offsets to find valid closure
            samples = [0.0, -self.overlap_distance, self.overlap_distance,
                       -self.overlap_distance * 0.5, self.overlap_distance * 0.5]
            found = False

            for s in samples:
                # Compute last_start with offset
                last_start = Point(prev_pt.x + prev_dir_unit.x * s,
                                   prev_pt.y + prev_dir_unit.y * s,
                                   prev_pt.z + prev_dir_unit.z * s)

                # Vector from last_start to root
                vec_to_root = Vector.from_start_end(last_start, root)
                dist_to_root = vec_to_root.length
                if dist_to_root == 0:
                    final_length = 0.0
                    final_end = root
                else:
                    # Constrain final length
                    max_len = self.stick_length + self.last_length_limit
                    min_len = max(0.0, self.stick_length - self.last_length_limit)
                    final_length = min(dist_to_root, max_len)
                    final_length = max(final_length, min_len)
                    unit_vec = Vector(vec_to_root.x / dist_to_root,
                                      vec_to_root.y / dist_to_root,
                                      vec_to_root.z / dist_to_root)
                    final_end = Point(
                        last_start.x + unit_vec.x * final_length,
                        last_start.y + unit_vec.y * final_length,
                        last_start.z + unit_vec.z * final_length
                    )

                # Check closure tolerance
                if distance_point_point(final_end, root) <= self.closure_tol:
                    # Build anchor points sequence
                    anchor_pts = pts[:-1]
                    anchor_pts.append(last_start)
                    anchor_pts.append(final_end)
                    
                    # Compute axes with overlap
                    axes = []
                    for i in range(len(anchor_pts) - 1):
                        a = anchor_pts[i]
                        b = anchor_pts[i + 1]
                        
                        if i == 0:
                            start_pt = a
                        else:
                            prev_a = anchor_pts[i - 1]
                            prev_dir = Vector.from_start_end(prev_a, a)
                            plen = prev_dir.length if prev_dir.length != 0 else 1.0
                            prev_unit = Vector(prev_dir.x / plen, prev_dir.y / plen, prev_dir.z / plen)
                            start_pt = Point(a.x - prev_unit.x * self.overlap_distance,
                                             a.y - prev_unit.y * self.overlap_distance,
                                             a.z - prev_unit.z * self.overlap_distance)
                        end_pt = b
                        axes.append((start_pt, end_pt))
                    
                    # Collision check
                    if self._segments_too_close(axes):
                        continue
                    
                    # Success
                    self.points = [pt for pt in anchor_pts]
                    self._temp_axes = axes
                    found = True
                    break

            if found:
                return True
        
        return False

    # ----------------------------
    def _build_sticks_from_lines(self):
        """Use self._temp_axes to make Stick objects with alternating z offsets in 3D."""
        self.lines = []
        self.sticks = []
        half = self.depth / 2.0
        up = True
        
        for idx, (s_pt, e_pt) in enumerate(self._temp_axes):
            # Get segment direction
            vec = e_pt - s_pt
            dir_unit = vec.unitized()

            # Apply offset along direction
            start_off = Point(s_pt.x - dir_unit.x * self.offset,
                                s_pt.y - dir_unit.y * self.offset,
                                s_pt.z - dir_unit.z * self.offset)
            end_off = Point(e_pt.x + dir_unit.x * self.offset,
                                e_pt.y + dir_unit.y * self.offset,
                                e_pt.z + dir_unit.z * self.offset)
            
            # Calculate perpendicular vector for z-offset (relative to stick direction)
            # Use cross product with global Z or another reference
            if abs(dir_unit.z) < 0.9:
                ref_vec = Vector(0, 0, 1)
            else:
                ref_vec = Vector(1, 0, 0)
            
            perp = dir_unit.cross(ref_vec)
            if perp.length > 0:
                zdir = perp.unitized() if up else perp.unitized().scaled(-1)
            else:
                zdir = Vector(0, 0, 1) if up else Vector(0, 0, -1)
            
            # Apply depth offset perpendicular to stick
            offs = Vector(zdir.x * half, zdir.y * half, zdir.z * half)
            start_off = Point(start_off.x + offs.x, start_off.y + offs.y, start_off.z + offs.z)
            end_off = Point(end_off.x + offs.x, end_off.y + offs.y, end_off.z + offs.z)
            
            line = Line(start_off, end_off)
            self.lines.append(line)
            stick = Stick(line, z_vector=zdir, width=self.width, depth=self.depth)
            self.sticks.append(stick)
            up = not up

    # ----------------------------
    def visualize(self):
        return [s.geometry for s in self.sticks]   

class LoopBranchModule3DWithFaces:
    """
    Generate a closed 3D loop where sticks connect face-to-face,
    combining angular rotation approach with face selection.
    """

    def __init__(self,
                 root_frame,
                 stick_length=600,
                 num_segments=8,
                 min_turn=math.radians(20),
                 max_turn=math.radians(140),
                 face_sequence=None,
                 overlap_distance=20.0,
                 last_length_limit=50.0,
                 offset=0,
                 depth=None,
                 width=None,
                 min_distance=30.0,
                 closure_tol=1.0,
                 max_attempts=200,
                 rotation_mode='full3d'):
        self.root_frame = root_frame
        self.stick_length = float(stick_length)
        self.num_segments = int(num_segments)
        self.min_turn = float(min_turn)
        self.max_turn = float(max_turn)
        self.face_sequence = face_sequence  # list of face indices (0-3) for each connection
        self.overlap_distance = float(overlap_distance)
        self.last_length_limit = float(last_length_limit)
        self.offset = float(offset)
        self.depth = depth or Stick.DEPTH
        self.width = width or Stick.WIDTH
        self.min_distance = float(min_distance)
        self.closure_tol = float(closure_tol)
        self.max_attempts = int(max_attempts)
        self.rotation_mode = rotation_mode

        self.points = []
        self.lines = []
        self.sticks = []
        self._face_frames = []  # Store face frames for debugging

        # Try to build a valid loop
        ok = self._generate_valid_loop()
        if ok:
            self._build_sticks_from_lines()
        else:
            self.points = []
            self.lines = []
            self.sticks = []

    # ----------------------------
    def _random_angle(self):
        """Return random turning angle within bounds (signed)."""
        a = random.uniform(self.min_turn, self.max_turn)
        return a if random.random() < 0.5 else -a

    # ----------------------------
    def _random_face_index(self):
        """Return random face index (0-3)."""
        return random.randint(0, 3)

    # ----------------------------
    def _get_rotation_from_face(self, face_index):
        """
        Convert face index to rotation angle around the stick's axis.
        Face 0: 0°, Face 1: 90°, Face 2: 180°, Face 3: 270°
        """
        return face_index * math.pi / 2

    # ----------------------------
    def _random_rotation_axis(self, current_direction):
        """Generate a random rotation axis perpendicular to current direction."""
        if abs(current_direction.x) < 0.9:
            temp = Vector(1, 0, 0)
        else:
            temp = Vector(0, 1, 0)
        
        perp1 = current_direction.cross(temp)
        if perp1.length > 0:
            perp1 = perp1.unitized()
        else:
            perp1 = Vector(0, 0, 1)
        
        perp2 = current_direction.cross(perp1)
        if perp2.length > 0:
            perp2 = perp2.unitized()
        
        theta = random.uniform(0, 2 * math.pi)
        axis = Vector(
            perp1.x * math.cos(theta) + perp2.x * math.sin(theta),
            perp1.y * math.cos(theta) + perp2.y * math.sin(theta),
            perp1.z * math.cos(theta) + perp2.z * math.sin(theta)
        )
        
        return axis.unitized() if axis.length > 0 else perp1

    # ----------------------------
    def _segment_midpoint(self, a: Point, b: Point):
        return Point((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)

    # ----------------------------
    def _segments_too_close(self, segments):
        """Check if segment midpoints are too close."""
        mids = [self._segment_midpoint(s[0], s[1]) for s in segments]
        n = len(mids)
        for i in range(n):
            for j in range(i + 2, n):
                if i == 0 and j == n - 1:
                    continue
                if distance_point_point(mids[i], mids[j]) < self.min_distance:
                    return True
        return False

    # ----------------------------
    def _generate_valid_loop(self):
        """
        Generate a valid 3D loop using angular rotation but tracking face connections.
        """
        root = Point(*self.root_frame.point)
        
        for attempt in range(self.max_attempts):
            # Generate face sequence if not provided
            if self.face_sequence is None:
                faces = [self._random_face_index() for _ in range(self.num_segments - 1)]
            else:
                faces = list(self.face_sequence)
                # Pad with random if too short
                while len(faces) < self.num_segments - 1:
                    faces.append(self._random_face_index())
            
            # Generate 3D anchor points
            pts = [root]
            direction = self.root_frame.xaxis
            current_up = self.root_frame.yaxis  # Track "up" direction for face orientation
            
            for i in range(self.num_segments - 1):
                # Get turning angle
                angle = self._random_angle()
                
                # Choose rotation axis based on mode
                if self.rotation_mode == 'full3d':
                    rotation_axis = self._random_rotation_axis(direction)
                elif self.rotation_mode == 'constrained':
                    if random.random() < 0.7:
                        rotation_axis = self.root_frame.zaxis
                    else:
                        rotation_axis = self._random_rotation_axis(direction)
                else:
                    rotation_axis = self.root_frame.zaxis
                
                # First: rotate for the face selection (around stick axis)
                face_rotation = self._get_rotation_from_face(faces[i])
                if face_rotation != 0:
                    current_up = current_up.rotated(face_rotation, direction)
                
                # Second: apply the turning angle
                direction = direction.rotated(angle, rotation_axis)
                current_up = current_up.rotated(angle, rotation_axis)
                
                # Next anchor point
                nxt = Point(
                    pts[-1].x + direction.x * self.stick_length,
                    pts[-1].y + direction.y * self.stick_length,
                    pts[-1].z + direction.z * self.stick_length,
                )
                pts.append(nxt)

            # Close the loop (same as before)
            prev_pt = pts[-1]
            prev_dir_vec = Vector.from_start_end(pts[-2], pts[-1])
            if prev_dir_vec.length == 0:
                continue
            prev_dir_unit = Vector(prev_dir_vec.x / prev_dir_vec.length,
                                   prev_dir_vec.y / prev_dir_vec.length,
                                   prev_dir_vec.z / prev_dir_vec.length)

            samples = [0.0, -self.overlap_distance, self.overlap_distance,
                       -self.overlap_distance * 0.5, self.overlap_distance * 0.5]
            found = False

            for s in samples:
                last_start = Point(prev_pt.x + prev_dir_unit.x * s,
                                   prev_pt.y + prev_dir_unit.y * s,
                                   prev_pt.z + prev_dir_unit.z * s)

                vec_to_root = Vector.from_start_end(last_start, root)
                dist_to_root = vec_to_root.length
                if dist_to_root == 0:
                    final_length = 0.0
                    final_end = root
                else:
                    max_len = self.stick_length + self.last_length_limit
                    min_len = max(0.0, self.stick_length - self.last_length_limit)
                    final_length = min(dist_to_root, max_len)
                    final_length = max(final_length, min_len)
                    unit_vec = Vector(vec_to_root.x / dist_to_root,
                                      vec_to_root.y / dist_to_root,
                                      vec_to_root.z / dist_to_root)
                    final_end = Point(
                        last_start.x + unit_vec.x * final_length,
                        last_start.y + unit_vec.y * final_length,
                        last_start.z + unit_vec.z * final_length
                    )

                if distance_point_point(final_end, root) <= self.closure_tol:
                    anchor_pts = pts[:-1]
                    anchor_pts.append(last_start)
                    anchor_pts.append(final_end)
                    
                    # Compute axes with overlap
                    axes = []
                    for i in range(len(anchor_pts) - 1):
                        a = anchor_pts[i]
                        b = anchor_pts[i + 1]
                        
                        if i == 0:
                            start_pt = a
                        else:
                            prev_a = anchor_pts[i - 1]
                            prev_dir = Vector.from_start_end(prev_a, a)
                            plen = prev_dir.length if prev_dir.length != 0 else 1.0
                            prev_unit = Vector(prev_dir.x / plen, prev_dir.y / plen, prev_dir.z / plen)
                            start_pt = Point(a.x - prev_unit.x * self.overlap_distance,
                                             a.y - prev_unit.y * self.overlap_distance,
                                             a.z - prev_unit.z * self.overlap_distance)
                        end_pt = b
                        axes.append((start_pt, end_pt))
                    
                    if self._segments_too_close(axes):
                        continue
                    
                    # Success
                    self.points = [pt for pt in anchor_pts]
                    self._temp_axes = axes
                    self._used_faces = faces
                    found = True
                    break

            if found:
                return True
        
        return False

    # ----------------------------
    def _build_sticks_from_lines(self):
        """
        Build Stick objects from axes.
        Each stick's z-vector is determined by the face connection.
        """
        self.lines = []
        self.sticks = []
        
        # Initial up direction from root frame
        current_up = self.root_frame.yaxis
        
        for idx, (s_pt, e_pt) in enumerate(self._temp_axes):
            # Get segment direction
            vec = e_pt - s_pt
            dir_unit = vec.unitized()

            # Apply offset along direction
            start_off = Point(s_pt.x - dir_unit.x * self.offset,
                            s_pt.y - dir_unit.y * self.offset,
                            s_pt.z - dir_unit.z * self.offset)
            end_off = Point(e_pt.x + dir_unit.x * self.offset,
                          e_pt.y + dir_unit.y * self.offset,
                          e_pt.z + dir_unit.z * self.offset)
            
            # For face connection: the z-vector should be perpendicular to stick direction
            # and in the plane determined by current_up
            # Project current_up onto plane perpendicular to dir_unit
            dot = current_up.x * dir_unit.x + current_up.y * dir_unit.y + current_up.z * dir_unit.z
            projected_up = Vector(
                current_up.x - dot * dir_unit.x,
                current_up.y - dot * dir_unit.y,
                current_up.z - dot * dir_unit.z
            )
            
            if projected_up.length > 0.01:
                zdir = projected_up.unitized()
            else:
                # Fallback: use cross product
                if abs(dir_unit.z) < 0.9:
                    ref_vec = Vector(0, 0, 1)
                else:
                    ref_vec = Vector(1, 0, 0)
                perp = dir_unit.cross(ref_vec)
                zdir = perp.unitized() if perp.length > 0 else Vector(0, 0, 1)
            
            # Apply depth offset perpendicular to stick
            half = self.depth / 2.0
            offs = Vector(zdir.x * half, zdir.y * half, zdir.z * half)
            start_off = Point(start_off.x + offs.x, start_off.y + offs.y, start_off.z + offs.z)
            end_off = Point(end_off.x + offs.x, end_off.y + offs.y, end_off.z + offs.z)
            
            # Update current_up for next stick based on face index
            if idx < len(self._used_faces):
                face_idx = self._used_faces[idx]
                face_rotation = self._get_rotation_from_face(face_idx)
                current_up = current_up.rotated(face_rotation, dir_unit)
            
            line = Line(start_off, end_off)
            self.lines.append(line)
            stick = Stick(line, z_vector=zdir, width=self.width, depth=self.depth)
            self.sticks.append(stick)

    # ----------------------------
    def visualize(self):
        return [s.geometry for s in self.sticks]
    
    # ----------------------------
    def get_face_sequence(self):
        """Return the face sequence used in the successful loop."""
        if hasattr(self, '_used_faces'):
            return self._used_faces
        return None

class FaceConnectedLoop3D:
    """
    Creates a 3D closed loop where sticks connect face-to-face with flat gluing surfaces.
    Each stick grows from a face of the previous stick, ensuring proper alignment for robotic gluing.
    """

    def __init__(self,
                 root_frame,
                 num_segments=8,
                 stick_length=None,
                 width=None,
                 depth=None,
                 face_sequence=None,
                 angle_sequence=None,
                 overlap_distance=10.0,
                 max_attempts=1000):
        """
        Parameters
        ----------
        root_frame : Frame
            Starting frame for the first stick.
        num_segments : int
            Number of sticks in the loop.
        face_sequence : list of int, optional
            Which face (0-3) to use for each connection. If None, random.
        angle_sequence : list of float, optional
            Rotation angle (radians) around face normal for each connection. If None, random.
        overlap_distance : float
            How much sticks overlap at joints.
        """
        self.root_frame = root_frame
        self.num_segments = int(num_segments)
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.overlap_distance = float(overlap_distance)
        self.max_attempts = int(max_attempts)
        
        self.sticks = []
        self.face_sequence = face_sequence
        self.angle_sequence = angle_sequence
        
        # Try to build a valid loop
        ok = self._generate_valid_loop()
        if not ok:
            print("Failed to generate valid loop after {} attempts".format(max_attempts))
            self.sticks = []

    # ----------------------------
    def _get_face_frame(self, stick, face_index):
        """
        Gets a frame on one of the four faces of a stick at its END point.
        This frame represents where the next stick can attach.
        
        Args:
            stick: Stick object
            face_index: Face index (0-3) around the stick
            
        Returns:
            Frame on the specified face, positioned for next stick growth
        """
        # Get stick's frame at the end
        stick_frame = Frame(stick.axis.end, stick.frame.xaxis, stick.frame.yaxis)
        
        # Rotate around stick's xaxis (length direction) based on face index
        # Face 0: yaxis (top), Face 1: -zaxis (right), Face 2: -yaxis (bottom), Face 3: zaxis (left)
        angle = face_index * math.pi / 2
        R = Rotation.from_axis_and_angle(stick_frame.xaxis, angle=angle, point=stick_frame.point)
        face_frame = stick_frame.transformed(R)
        
        # Move frame to the surface of the stick (offset by depth/2 along face normal)
        face_frame.point = face_frame.point + face_frame.yaxis * (self.depth / 2)
        
        return face_frame

    # ----------------------------
    def _create_next_stick_from_face(self, prev_stick, face_index, rotation_angle):
        """
        Creates the next stick growing from a specific face of the previous stick.
        The connection will be a flat face-to-face joint suitable for gluing.
        
        Args:
            prev_stick: Previous Stick object
            face_index: Which face (0-3) to grow from
            rotation_angle: Additional rotation around the face normal (radians)
            
        Returns:
            New Stick object
        """
        # Get the face frame where we'll attach
        face_frame = self._get_face_frame(prev_stick, face_index)
        
        # Apply additional rotation around face normal (yaxis of face_frame)
        if rotation_angle != 0:
            R = Rotation.from_axis_and_angle(face_frame.yaxis, rotation_angle, point=face_frame.point)
            face_frame.transform(R)
        
        # Position for new stick: move inward by depth/2 and back by overlap
        new_start = face_frame.point + face_frame.yaxis * (self.depth / 2)
        new_start = new_start + face_frame.xaxis * (-self.overlap_distance)
        
        # Create the new stick's centerline
        # The stick grows along face_frame.xaxis
        new_end = new_start + face_frame.xaxis * self.stick_length
        centerline = Line(new_start, new_end)
        
        # The z-vector of the new stick should be the face normal (face_frame.yaxis)
        new_stick = Stick(centerline, z_vector=face_frame.yaxis, width=self.width, depth=self.depth)
        
        return new_stick

    # ----------------------------
    def _check_closure_possible(self, last_stick, target_frame, tolerance=50.0):
        """
        Check if we can close the loop from the last stick back to the target frame.
        
        Returns:
            Tuple of (can_close: bool, best_face: int, best_angle: float, distance: float)
        """
        best_distance = float('inf')
        best_face = None
        best_angle = None
        
        # Try all faces
        for face_idx in range(4):
            # Try different rotation angles
            for angle in [0, math.pi/6, -math.pi/6, math.pi/4, -math.pi/4, math.pi/3, -math.pi/3]:
                face_frame = self._get_face_frame(last_stick, face_idx)
                
                # Apply rotation
                if angle != 0:
                    R = Rotation.from_axis_and_angle(face_frame.yaxis, angle, point=face_frame.point)
                    face_frame.transform(R)
                
                # Calculate where a stick from this face would end
                new_start = face_frame.point + face_frame.yaxis * (self.depth / 2)
                new_start = new_start + face_frame.xaxis * (-self.overlap_distance)
                
                # Vector toward target
                vec_to_target = Vector.from_start_end(new_start, target_frame.point)
                dist_to_target = vec_to_target.length
                
                # Check if this direction aligns with face_frame.xaxis
                if dist_to_target > 0.1:
                    dir_to_target = vec_to_target.unitized()
                    alignment = face_frame.xaxis.dot(dir_to_target)
                    
                    # Good alignment and reasonable distance?
                    if alignment > 0.85:  # Mostly aligned
                        # Check if distance matches stick length reasonably
                        length_diff = abs(dist_to_target - self.stick_length)
                        
                        if length_diff < best_distance:
                            best_distance = length_diff
                            best_face = face_idx
                            best_angle = angle
        
        if best_face is not None and best_distance < tolerance:
            return True, best_face, best_angle, best_distance
        
        return False, None, None, float('inf')

    # ----------------------------
    def _generate_valid_loop(self):
        """
        Generate a valid closed loop with face-to-face connections.
        """
        for attempt in range(self.max_attempts):
            sticks = []
            
            # Create first stick from root frame
            first_centerline = Line.from_point_and_vector(
                self.root_frame.point,
                self.root_frame.xaxis * self.stick_length
            )
            first_stick = Stick(first_centerline, z_vector=self.root_frame.yaxis,
                              width=self.width, depth=self.depth)
            sticks.append(first_stick)
            
            # Generate or use provided sequences
            if self.face_sequence is not None:
                faces = list(self.face_sequence)
            else:
                faces = [random.randint(0, 3) for _ in range(self.num_segments - 2)]
            
            if self.angle_sequence is not None:
                angles = list(self.angle_sequence)
            else:
                angles = [random.uniform(-math.pi/4, math.pi/4) for _ in range(self.num_segments - 2)]
            
            # Ensure we have enough values
            while len(faces) < self.num_segments - 2:
                faces.append(random.randint(0, 3))
            while len(angles) < self.num_segments - 2:
                angles.append(random.uniform(-math.pi/4, math.pi/4))
            
            # Build intermediate sticks
            success = True
            for i in range(self.num_segments - 2):
                try:
                    next_stick = self._create_next_stick_from_face(
                        sticks[-1], 
                        faces[i], 
                        angles[i]
                    )
                    sticks.append(next_stick)
                except Exception as e:
                    success = False
                    break
            
            if not success:
                continue
            
            # Try to close the loop
            can_close, close_face, close_angle, close_dist = self._check_closure_possible(
                sticks[-1], 
                self.root_frame,
                tolerance=self.stick_length * 0.3
            )
            
            if can_close:
                # Create final closing stick
                try:
                    # Create a stick that reaches back to root
                    face_frame = self._get_face_frame(sticks[-1], close_face)
                    
                    if close_angle != 0:
                        R = Rotation.from_axis_and_angle(face_frame.yaxis, close_angle, point=face_frame.point)
                        face_frame.transform(R)
                    
                    new_start = face_frame.point + face_frame.yaxis * (self.depth / 2)
                    new_start = new_start + face_frame.xaxis * (-self.overlap_distance)
                    
                    # End at root frame's starting point
                    new_end = self.root_frame.point
                    
                    centerline = Line(new_start, new_end)
                    final_stick = Stick(centerline, z_vector=face_frame.yaxis, 
                                      width=self.width, depth=self.depth)
                    
                    sticks.append(final_stick)
                    
                    # Success!
                    self.sticks = sticks
                    self._successful_faces = faces[:self.num_segments-2] + [close_face]
                    self._successful_angles = angles[:self.num_segments-2] + [close_angle]
                    return True
                    
                except Exception as e:
                    continue
        
        return False

    # ----------------------------
    def visualize(self):
        """Returns geometry of all sticks."""
        return [s.geometry for s in self.sticks]

    # ----------------------------
    def get_connection_info(self):
        """
        Returns information about face connections for robotic gluing.
        Each connection shows which face was used and the rotation angle.
        """
        if not hasattr(self, '_successful_faces'):
            return None
        
        info = []
        for i, (face, angle) in enumerate(zip(self._successful_faces, self._successful_angles)):
            info.append({
                'connection': i,
                'from_stick': i,
                'to_stick': i + 1,
                'face_index': face,
                'rotation_angle_deg': math.degrees(angle),
                'face_name': ['top', 'right', 'bottom', 'left'][face]
            })
        return info

class CurveLoopModule:
    def __init__(self, curve, segment_length, offset=0, width=None, depth=None):
        self.curve = curve
        self.segment_length = float(segment_length)
        self.offset = float(offset)
        self.width = width or 13.0
        self.depth = depth or 13.0
        self.points = []
        self.axes = []
        self.sticks = []
        self._process()
    
    def _process(self):
        if not self.curve:
            return
        
        self.divide_curve()
        self.make_axes()
        self.build_sticks()
    
    def divide_curve(self):
        params, pts = self.curve.divide_by_length(self.segment_length, return_points=True)
        self.points = pts
    
    def make_axes(self):
        self.axes = []
        num_pts = len(self.points)
        for i in range(num_pts - 1):
            start_pt = self.points[i]
            end_pt = self.points[i + 1]
            axis = Line(start_pt, end_pt)
            self.axes.append(axis)
    
    def build_sticks(self):
        self.sticks = []
        up = True
        half = self.depth / 2.0

        for axis in self.axes:
            # define z direction vector based on up/down
            vec = axis.vector
            length = vec.length
            if length == 0:
                continue
            dir_unit = vec.unitized()

            start_off = axis.start - dir_unit * self.offset
            end_off = axis.end + dir_unit * self.offset

            zdir = Vector(0, 0, 1) if up else Vector(0, 0, -1)
            zvec = zdir * half
            
            start_off = Point(start_off.x + zvec.x,
                              start_off.y + zvec.y,
                              start_off.z + zvec.z)
            end_off = Point(end_off.x + zvec.x,
                            end_off.y + zvec.y,
                            end_off.z + zvec.z)
            new_line = Line(start_off, end_off)

            # create stick with z_vector
            stick = Stick(new_line, z_vector=zdir, width=self.width, depth=self.depth)

            self.sticks.append(stick)
            up = not up  # alternate
    
    def get_frame_at_stick(self, stick_index, t=0.5):
        stick = self.sticks[stick_index]
        point_on_axis = stick.axis.point_at(t)
        frame = Frame(point_on_axis, stick.axis.direction, stick.z_vector)
        return frame
    
    def get_sticks(self):
        return self.sticks
    
    def visualize(self):
        return [s.geometry for s in self.sticks]
    

