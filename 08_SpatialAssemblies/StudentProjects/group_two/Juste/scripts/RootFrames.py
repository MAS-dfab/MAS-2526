# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point,
    Vector,
    Frame,
    Line,
    Box,
    Rotation,
    Transformation,
)

# =============================================================================
# DIAGNOSTICS (soft-mode logging)
# =============================================================================


class Diagnostics(object):
    def __init__(self):
        self.warnings = []

    def warn(self, msg):
        self.warnings.append(str(msg))

    def dump(self, prefix="[RootFrames] "):
        if not self.warnings:
            return
        print(prefix + "Diagnostics:")
        for w in self.warnings:
            print(prefix + "  - " + w)


# =============================================================================
# HELPERS
# =============================================================================


def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    if y.length < 1e-9:
        y = worldY
    y.unitize()
    return y


def _distance_point_segment(pt, line):
    """Approximate distance from a point to a line segment."""
    p0 = line.start
    p1 = line.end
    u = p1 - p0
    uu = u.dot(u)
    if uu < 1e-12:
        return pt.distance_to_point(p0)

    t = (pt - p0).dot(u) / uu
    if t <= 0.0:
        cp = p0
    elif t >= 1.0:
        cp = p1
    else:
        cp = p0 + u * t
    return pt.distance_to_point(cp)


def _segment_distance(line1, line2):
    """Sampled segment–segment distance (collision hint)."""
    p0 = line1.start
    p1 = line1.end
    m1 = (p0 + p1) * 0.5

    q0 = line2.start
    q1 = line2.end
    m2 = (q0 + q1) * 0.5

    pts1 = [p0, m1, p1]
    pts2 = [q0, m2, q1]

    dmin = 1e9
    for p in pts1:
        dmin = min(dmin, _distance_point_segment(p, line2))
    for q in pts2:
        dmin = min(dmin, _distance_point_segment(q, line1))
    return dmin


def _aabb_from_box(box):
    """Axis-aligned bounding box (world-space) from a compas Box."""
    # vertices is already a list-like property in compas 2.x
    verts = list(box.vertices)      #  <-- FIX: no parentheses

    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    return (
        min(xs), max(xs),
        min(ys), max(ys),
        min(zs), max(zs),
    )



def _aabb_overlap(a, b, clearance=0.0):
    """Check AABB–AABB overlap with optional clearance."""
    eps = float(clearance) * 0.5
    ax0, ax1, ay0, ay1, az0, az1 = a
    bx0, bx1, by0, by1, bz0, bz1 = b

    if ax1 + eps < bx0 - eps:
        return False
    if bx1 + eps < ax0 - eps:
        return False
    if ay1 + eps < by0 - eps:
        return False
    if by1 + eps < ay0 - eps:
        return False
    if az1 + eps < bz0 - eps:
        return False
    if bz1 + eps < az0 - eps:
        return False
    return True


# =============================================================================
# STICK
# =============================================================================


class Stick(object):
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None):
        """
        Parameters
        ----------
        axis : compas.geometry.Line
            Centerline of the stick.
        length : float, optional
        width : float, optional
        depth : float, optional
        """
        self.axis = axis
        self.length = length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self.compute_frame()

    def compute_frame(self):
        x = self.axis.direction.unitized()
        y = _stable_perp(x)
        # z is implied by Frame (x, y)
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return a compas Box aligned with the stick frame."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box

    @property
    def aabb(self):
        """Axis-aligned bounding box (tuple of 6 floats)."""
        return _aabb_from_box(self.geometry)


# =============================================================================
# BRANCHING MODULE  (3D, face-contact)
# =============================================================================


class BranchingModule(object):
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies on a parent face (full-width/depth offset).
      - Child axis is a blend of parent tangent and face normal.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None,
                 offset01=0.5, diag=None):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)
        self.diag = diag or Diagnostics()

    # -- local helpers ---------------------------------------------------

    def _face_normal_and_half(self, parent, face_index):
        """Return (normal, half_thickness) for given parent face index."""
        fi = int(face_index) % 4
        pf = parent.frame

        if fi == 0:           # +Y
            n = pf.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:         # -Y
            n = (-pf.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:         # +Z
            n = pf.zaxis.unitized()
            half = self.depth * 0.5
        else:                 # -Z
            n = (-pf.zaxis).unitized()
            half = self.depth * 0.5

        return n, half

    def _build_child_from_face(self, parent, face_index, stick_angle):
        """Create a child stick whose near face sits outside the parent face."""
        # clamp offset 0–1
        t = max(0.0, min(1.0, self.offset01))
        axis_base = parent.axis.point_at(t)

        n, half = self._face_normal_and_half(parent, face_index)

        # parent outer face center
        parent_face_center = axis_base + n * half
        # child near face center offset further by its own half-width/depth
        near_face_center = parent_face_center + n * half  # same dims

        # tangent from parent (axis direction)
        tangent = parent.frame.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = _stable_perp(n)
        tangent_proj.unitize()

        # blend normal & tangent via designer angle
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)

        # ensure direction not degenerate
        if d_raw.length < 1e-9:
            self.diag.warn("Degenerate branch direction; using tangent only.")
            d = tangent_proj
        else:
            d = d_raw.unitized()

        # enforce outward component (avoid pushing back into parent)
        if d.dot(n) <= 0.0:
            d = -d

        # build axis so that near end is at near_face_center
        start = near_face_center
        end = near_face_center + d * self.stick_length
        axis = Line(start, end)

        child = Stick(axis, length=self.stick_length,
                      width=self.width, depth=self.depth)
        return child

    # -- public API ------------------------------------------------------

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# BRIDGING MODULE  (Option C – maximum connectivity, soft)
# =============================================================================


class BridgingModule(object):
    """
    Very simple bridging between sticks, Option C (max connectivity):

    - Given a list of root sticks, attempt to connect every pair whose
      distance < max_dist and angle < max_angle.
    - Each bridge is a new Stick starting at one stick face and aiming
      roughly toward the midpoint between the two parent sticks.

    This is intentionally conservative & soft: if anything looks sketchy,
    the bridge is skipped and a warning is logged instead of exploding.
    """

    def __init__(self, sticks, stick_length=None, width=None, depth=None,
                 max_dist=None, max_angle=135.0, diag=None):
        self.parents = list(sticks)
        self.len = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.max_dist = max_dist  # if None, computed from bounds
        self.max_angle = float(max_angle)
        self.diag = diag or Diagnostics()
        self.bridges = []

        if self.max_dist is None and self.parents:
            # rough heuristic: average stick length
            avg_len = sum(s.axis.length for s in self.parents) / len(self.parents)
            self.max_dist = 1.5 * avg_len

    # simple helper
    def _center(self, stick):
        return stick.axis.midpoint

    def _try_bridge_pair(self, s0, s1):
        c0 = self._center(s0)
        c1 = self._center(s1)
        v01 = Vector.from_start_end(c0, c1)
        d = v01.length
        if d < 1e-6:
            self.diag.warn("Bridge pair too close; skipping.")
            return None
        if d > self.max_dist:
            return None

        # angle filter
        dir0 = s0.frame.xaxis
        dir1 = s1.frame.xaxis
        ang = math.degrees(dir0.angle(dir1))
        if ang > self.max_angle:
            return None

        # bridge direction from s0 toward midpoint
        mid = c0 + v01 * 0.5
        direction = Vector.from_start_end(c0, mid)
        if direction.length < 1e-6:
            self.diag.warn("Bridge direction degenerate; skipping pair.")
            return None
        direction.unitize()

        start = c0
        end = c0 + direction * self.len
        axis = Line(start, end)

        return Stick(axis, length=self.len, width=self.width, depth=self.depth)

    def build_bridges(self):
        n = len(self.parents)
        if n < 2:
            return []

        for i in range(n):
            for j in range(i + 1, n):
                s0 = self.parents[i]
                s1 = self.parents[j]
                br = self._try_bridge_pair(s0, s1)
                if br:
                    self.bridges.append(br)

        return self.bridges


# =============================================================================
# ROOTFRAMES ENGINE (3D, double-curved aware)
# =============================================================================


class RootFrames(object):
    """
    Pipeline for 3D growth:
      1) Surface/Curve → sample points in 3D
      2) Points → frames using true surface/curve frames (3D normals)
      3) Frames → edge frames + edge directions (no flattening)
      4) Growth: branching with BranchingModule
      5) Optional: bridging with BridgingModule (Option C)
      6) Optional: collision detection (AABB + segment fallback)
    """

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=10,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        self.diag = Diagnostics()

        self.surface_input = surface  # Rhino surface/Brep or None
        self.curve_input = curve      # Rhino curve or None

        self.point_density = int(point_density)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width = stick_width or Stick.WIDTH
        self.stick_depth = stick_depth or Stick.DEPTH

        # data containers
        self.points = []         # [compas Point]
        self.frames = []         # root frames on geometry
        self.edge_frames = []    # frames along edges
        self.edge_vectors = []   # edge directions (3D)
        self.edges = []          # list of (i, j) index pairs
        self.sticks = []         # resulting Stick objects (branches + bridges)
        self.collision_flags = []

        # sampling parameters (for 3D frames)
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ------------------------------------------------------------------  #
    # 1. SAMPLING
    # ------------------------------------------------------------------  #

    def _sample_points(self):
        """Sample points on a 3D curve or surface."""
        self.points = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        import Rhino.Geometry as rg

        pts = []

        # ---- Curve mode -------------------------------------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv
            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            count = max(1, self.point_density)
            for _ in range(count):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # ---- Surface / Brep mode ---------------------------------------
        else:
            if self.surface_input is None:
                raise Exception("No surface_input for sampling (RootFrames).")

            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("surface_input.ToBrep() has no faces.")

            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            count = max(1, self.point_density)
            for _ in range(count):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------  #
    # 2. FRAMES FROM GEOMETRY (FULL 3D)
    # ------------------------------------------------------------------  #

    def _frames_from_geometry(self, rot_tan=0.0, rot_norm=0.0):
        """Build 3D frames using Rhino's frame evaluation."""
        N = len(self.points)
        if N == 0:
            self.frames = []
            return []

        import Rhino.Geometry as rg

        frames = []

        # ---- Curve mode -------------------------------------------------
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    # fallback: tangent frame from derivative
                    tangent = crv.TangentAt(t)
                    tvec = Vector(tangent.X, tangent.Y, tangent.Z)
                    if tvec.length < 1e-6:
                        tvec = Vector(1, 0, 0)
                    tvec.unitize()
                    y = _stable_perp(tvec)
                    f = Frame(pt, tvec, y)
                    self.diag.warn("Curve.FrameAt failed; using tangent frame.")
                else:
                    xaxis = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    yaxis = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    if xaxis.length < 1e-6:
                        xaxis = Vector(1, 0, 0)
                    else:
                        xaxis.unitize()
                    if yaxis.length < 1e-6:
                        yaxis = _stable_perp(xaxis)
                    else:
                        yaxis.unitize()
                    f = Frame(pt, xaxis, yaxis)

                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        # ---- Surface mode -----------------------------------------------
        elif self._rg_face is not None and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)
                if not ok:
                    f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                    self.diag.warn("Face.FrameAt failed; using world frame.")
                else:
                    xaxis = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    yaxis = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    if xaxis.length < 1e-6:
                        xaxis = Vector(1, 0, 0)
                    else:
                        xaxis.unitize()
                    if yaxis.length < 1e-6:
                        yaxis = _stable_perp(xaxis)
                    else:
                        yaxis.unitize()
                    f = Frame(pt, xaxis, yaxis)

                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        else:
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                frames.append(f)
                self.diag.warn("No geometry frame data; using world frame.")

        self.frames = frames
        return frames

    # ------------------------------------------------------------------  #
    # 3. EDGE FRAMES & VECTORS (NO FLATTENING)
    # ------------------------------------------------------------------  #

    def _frames_to_edgevectors(self):
        """Nearest-neighbour edges with full 3D directions."""
        pts = [f.point for f in self.frames]
        N = len(pts)

        if N < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            return [], []

        edges = set()
        for i in range(N):
            pi = pts[i]
            best = 1e9
            j_best = None
            for j in range(N):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j
            if j_best is not None:
                edges.add(tuple(sorted((i, j_best))))

        edges = [(i, j) for (i, j) in edges if i < N and j < N]
        self.edges = edges

        eframes = []
        evectors = []

        for i, j in edges:
            f = self.frames[i]
            p0 = f.point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                self.diag.warn("Zero-length edge; skipping.")
                continue

            x = v.unitized()
            y = _stable_perp(x)
            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------------  #
    # 4. GROWTH (BRANCHING + optional BRIDGING)
    # ------------------------------------------------------------------  #

    def _grow_sticks_branching(self, steps=1, face_index=0,
                               stick_angle=0.0, offset01=0.5):
        """Create root sticks on edges, then branch from each root."""
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            self.diag.warn("No edge frames to grow from.")
            return sticks_out

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(axis, length=self.stick_length,
                         width=self.stick_width,
                         depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        for r in roots:
            mod = BranchingModule(
                r,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
                diag=self.diag,
            )
            mod.grow_chain(
                steps=steps,
                face_index=face_index,
                stick_angle=stick_angle,
            )
            sticks_out.extend(mod.sticks[1:])  # skip duplicated root

        return sticks_out

    def _grow_sticks_bridging(self, parents,
                              max_dist=None, max_angle=135.0):
        """Optional bridging phase (Option C)."""
        mod = BridgingModule(
            parents,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
            max_dist=max_dist,
            max_angle=max_angle,
            diag=self.diag,
        )
        bridges = mod.build_bridges()
        return bridges

    # ------------------------------------------------------------------  #
    # 5. COLLISION DETECTION
    # ------------------------------------------------------------------  #

    def detect_collisions(self, clearance=0.0):
        """Flag sticks whose boxes overlap (AABB) or whose centerlines are too close."""
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        # precompute AABBs
        aabbs = [s.aabb for s in self.sticks]
        base_thick = max(self.stick_width, self.stick_depth)

        for i in range(n):
            for j in range(i + 1, n):
                a = aabbs[i]
                b = aabbs[j]
                if not _aabb_overlap(a, b, clearance=clearance):
                    continue
                # if AABBs overlap, refine with segment distance
                d = _segment_distance(self.sticks[i].axis, self.sticks[j].axis)
                if d < base_thick + clearance:
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------------  #
    # 6. RUN
    # ------------------------------------------------------------------  #

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        clearance=0.0,
        rot_tan=0.0,
        rot_norm=0.0,
        do_bridging=False,
        bridge_max_dist=None,
        bridge_max_angle=135.0,
    ):
        """
        Execute the full RootFrames pipeline.

        Parameters match your GH integrator defaults; extra options
        (bridging, rotations) are optional and safe to ignore.
        """
        # 1–3: geometry → frames → edges
        self._sample_points()
        self._frames_from_geometry(rot_tan=rot_tan, rot_norm=rot_norm)
        self._frames_to_edgevectors()

        # 4a: branching
        branch_sticks = self._grow_sticks_branching(
            steps=steps,
            face_index=0,          # could be exposed later
            stick_angle=stick_angle,
            offset01=offset01,
        )

        all_sticks = list(branch_sticks)

        # 4b: bridging (optional, off by default)
        if do_bridging and branch_sticks:
            bridges = self._grow_sticks_bridging(
                parents=branch_sticks,
                max_dist=bridge_max_dist,
                max_angle=bridge_max_angle,
            )
            all_sticks.extend(bridges)

        self.sticks = all_sticks

        # 5: collisions (soft mode)
        if detect_collisions:
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * len(self.sticks)

        # 6: dump diagnostics (soft warnings)
        self.diag.dump()

        return self.sticks
