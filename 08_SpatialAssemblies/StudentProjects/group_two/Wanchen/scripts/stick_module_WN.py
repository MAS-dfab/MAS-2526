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
    

