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
# HELPERS
# =============================================================================


def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    if y.length < 1e-6:
        y = worldY
    y.unitize()
    return y


def _aabb_from_box(box):
    """Axis-aligned bounding box from a compas Box (already transformed)."""
    verts = list(box.vertices)
    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    pmin = Point(min(xs), min(ys), min(zs))
    pmax = Point(max(xs), max(ys), max(zs))
    return pmin, pmax


def _aabb_overlap(a_min, a_max, b_min, b_max, clearance=0.0):
    """Return True if AABBs overlap (with optional clearance)."""
    c = float(clearance)

    if a_max.x + c < b_min.x or b_max.x + c < a_min.x:
        return False
    if a_max.y + c < b_min.y or b_max.y + c < a_min.y:
        return False
    if a_max.z + c < b_min.z or b_max.z + c < a_min.z:
        return False
    return True


# =============================================================================
# STICK
# =============================================================================


class Stick:
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
        z = x.cross(y).unitized()
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return a compas Box aligned with the stick frame."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# =============================================================================
# BRANCHING MODULE  (3D, face-contact)
# =============================================================================


class BranchingModule:
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies on a parent face (full-width/depth offset).
      - Child axis is a blend of parent tangent and face normal (3D, no flattening).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    # ------------------------------------------------------------------

    def _build_child_from_face(self, parent, face_index, stick_angle):
        """
        Build a child whose near face has TRUE contact with a chosen parent face,
        while allowing full 3D branching (no projection back to the generator plane).
        """
        fi = int(face_index) % 4
        pf = parent.frame

        # position along parent axis (0–1)
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # face normal + half-thickness (parent & child share dims)
        if fi == 0:  # +Y
            n = pf.yaxis.unitized()
            parent_half = self.width * 0.5
            child_half = self.width * 0.5
        elif fi == 2:  # –Y
            n = (-pf.yaxis).unitized()
            parent_half = self.width * 0.5
            child_half = self.width * 0.5
        elif fi == 1:  # +Z
            n = pf.zaxis.unitized()
            parent_half = self.depth * 0.5
            child_half = self.depth * 0.5
        else:  # –Z
            n = (-pf.zaxis).unitized()
            parent_half = self.depth * 0.5
            child_half = self.depth * 0.5

        # parent outer surface
        parent_face_center = axis_pt + n * parent_half

        # full 3D direction: blend normal + tangent (no projection to plane)
        tangent = pf.xaxis
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tangent * math.sin(theta)
        if d_raw.length < 1e-6:
            d_raw = tangent
        d = d_raw.unitized()

        # child center: move off parent face by child_half, then along axis by half-length
        half_len = self.stick_length * 0.5
        child_center = parent_face_center + n * child_half + d * half_len

        # construct child frame & axis
        x = d
        y = n
        z = x.cross(y).unitized()
        child_frame = Frame(child_center, x, y)

        start = child_center - x * half_len
        end = child_center + x * half_len
        axis = Line(start, end)

        child = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        child.frame = child_frame
        return child

    # ------------------------------------------------------------------

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# ROOTFRAMES ENGINE (3D, double-curved aware, branching + AABB collisions)
# =============================================================================


class RootFrames:
    """
    Pipeline for 3D growth:
      1) Surface/Curve → sample points in 3D
      2) Points → frames using true surface/curve frames (3D normals)
      3) Frames → edge frames + edge directions (no flattening)
      4) Growth: branching with BranchingModule
      5) Collision detection: AABB from actual Box geometry
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
        self.sticks = []         # resulting Stick objects
        self.collision_flags = []

        # sampling parameters (for 3D frames)
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ----------------------------------------------------------------------
    # 1. SAMPLING
    # ----------------------------------------------------------------------

    def _sample_points(self):
        """Sample points on a 3D curve or (possibly double-curved) surface."""
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

            # keep sort by Z for consistency (not strictly required)
            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAMES FROM GEOMETRY (FULL 3D)
    # ----------------------------------------------------------------------

    def _frames_from_geometry(self, rot_tan=0.0, rot_norm=0.0):
        """Build 3D frames using Rhino's curve/surface frame evaluation."""
        N = len(self.points)
        if N == 0:
            self.frames = []
            return []

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

                # optional rotations about tangent / normal
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
            # last-resort fallback
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                frames.append(f)

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. EDGE FRAMES & VECTORS (NO FLATTENING)
    # ----------------------------------------------------------------------

    def _frames_to_edgevectors(self):
        """Nearest-neighbour edges with full 3D directions (no projection)."""
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
                continue

            x = v.unitized()
            y = _stable_perp(x)
            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ----------------------------------------------------------------------
    # 4. GROWTH (BRANCHING)
    # ----------------------------------------------------------------------

    def _grow_sticks_branching(self, steps=1, face_index=0, stick_angle=0.0, offset01=0.5):
        """Create root sticks on edges, then branch from each root."""
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            return sticks_out

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(axis, length=self.stick_length, width=self.stick_width, depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        for r in roots:
            mod = BranchingModule(
                r,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )
            mod.grow_chain(
                steps=steps,
                face_index=face_index,
                stick_angle=stick_angle,
            )
            sticks_out.extend(mod.sticks[1:])  # skip duplicated root

        self.sticks = sticks_out
        return sticks_out

    # ----------------------------------------------------------------------
    # 5. COLLISION DETECTION (AABB on Box geometry)
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        """
        Flag sticks whose bounding boxes intersect (AABB on transformed Boxes).
        """
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        # precompute all boxes and AABBs
        boxes = [s.geometry for s in self.sticks]
        aabbs = [_aabb_from_box(b) for b in boxes]

        for i in range(n):
            a_min, a_max = aabbs[i]
            for j in range(i + 1, n):
                b_min, b_max = aabbs[j]
                if _aabb_overlap(a_min, a_max, b_min, b_max, clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ----------------------------------------------------------------------
    # 6. RUN
    # ----------------------------------------------------------------------

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        clearance=0.0,
        rot_tan=0.0,
        rot_norm=0.0,
    ):
        """
        Execute the full RootFrames pipeline (3D branching + optional collisions).
        """
        self._sample_points()
        self._frames_from_geometry(rot_tan=rot_tan, rot_norm=rot_norm)
        self._frames_to_edgevectors()

        sticks = self._grow_sticks_branching(
            steps=steps,
            face_index=0,          # can be exposed later
            stick_angle=stick_angle,
            offset01=offset01,
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * len(self.sticks)

        return sticks
