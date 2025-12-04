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
# BRANCHING MODULE  (L-system style, face-contact)
# =============================================================================


class BranchingModule:
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies on a parent face (full-width/depth offset).
      - Child axis is a blend of parent tangent and face normal.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    def _build_child_from_face(self, parent, face_index, stick_angle):
        fi = int(face_index) % 4
        pf = parent.frame

        # position along parent axis (0–1)
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # pick face normal & thickness (parent & child share dims)
        if fi == 0:  # +Y
            n = pf.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:  # -Y
            n = (-pf.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:  # +Z
            n = pf.zaxis.unitized()
            half = self.depth * 0.5
        else:  # -Z
            n = (-pf.zaxis).unitized()
            half = self.depth * 0.5

        # parent face center
        parent_face_center = axis_pt + n * half

        # child center so that its near face coincides with parent face
        child_center = parent_face_center + n * half

        # tangent direction projected off the normal
        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = _stable_perp(n)
        tangent_proj.unitize()

        # blend normal & projected tangent with designer angle
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)

        # reproject to plane orthogonal to n (for a clean face contact)
        d = d_raw - n * d_raw.dot(n)
        if d.length < 1e-6:
            d = tangent_proj
        d.unitize()

        x = d
        y = n
        z = x.cross(y).unitized()
        child_frame = Frame(child_center, x, y)

        half_len = self.stick_length * 0.5
        start = child_center - x * half_len
        end = child_center + x * half_len
        axis = Line(start, end)

        child = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        child.frame = child_frame
        return child

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# ROOTFRAMES ENGINE (3D, double-curved aware)
# =============================================================================


class RootFrames:
    """
    Pipeline for 3D growth:
      1) Surface/Curve → sample points in 3D
      2) Points → frames using true surface/curve frames (3D normals)
      3) Frames → edge frames + edge directions (no flattening)
      4) Growth: branching with BranchingModule
      5) Optional: collision detection (approximate)
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
        """Sample points on a 3D curve or double-curved surface."""
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
            for i in range(count):
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

            # sorting by Z is optional for surfaces; keep it for consistency
            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAMES FROM GEOMETRY (FULL 3D)
    # ----------------------------------------------------------------------

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

                # optional rotations in local 3D frame
                if rot_tan:
                    R = Transformation.from_axis_and_angle(f.xaxis, math.radians(rot_tan))
                    f.transform(R)
                if rot_norm:
                    R = Transformation.from_axis_and_angle(f.yaxis, math.radians(rot_norm))
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
                    R = Transformation.from_axis_and_angle(f.xaxis, math.radians(rot_tan))
                    f.transform(R)
                if rot_norm:
                    R = Transformation.from_axis_and_angle(f.yaxis, math.radians(rot_norm))
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
    # 4. GROWTH (BRANCHING ONLY, FOR NOW)
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
    # 5. COLLISION DETECTION (APPROX)
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        """Flag sticks whose centerlines come closer than their thickness."""
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        base_thick = max(self.stick_width, self.stick_depth) + float(clearance)

        for i in range(n):
            li = self.sticks[i].axis
            for j in range(i + 1, n):
                lj = self.sticks[j].axis
                d = _segment_distance(li, lj)
                if d < base_thick:
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
        """Execute the full RootFrames pipeline (branching only)."""
        self._sample_points()
        self._frames_from_geometry(rot_tan=rot_tan, rot_norm=rot_norm)
        self._frames_to_edgevectors()

        sticks = self._grow_sticks_branching(
            steps=steps,
            face_index=0,          # you can expose this later
            stick_angle=stick_angle,
            offset01=offset01,
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)

        return sticks
