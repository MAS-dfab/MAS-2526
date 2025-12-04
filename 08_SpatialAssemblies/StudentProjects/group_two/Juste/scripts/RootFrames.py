# RootFrames.py
# clean, coherent, compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box,
    Plane, Rotation, Transformation
)

# ============================================================
# HELPERS
# ============================================================

def _stable_perp(v):
    """Return a stable perpendicular vector."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(v.dot(worldZ)) < 0.9 else worldY
    y = up.cross(v)
    y.unitize()
    return y


# simple AABB from center and axes
def stick_aabb(stick):
    f = stick.frame
    c = f.point

    lx = stick.length * 0.5
    ly = stick.width  * 0.5
    lz = stick.depth  * 0.5

    # extremal points in local axes
    corners = []
    for sx in (-lx, lx):
        for sy in (-ly, ly):
            for sz in (-lz, lz):
                p = c + f.xaxis * sx + f.yaxis * sy + f.zaxis * sz
                corners.append(p)

    xs = [p.x for p in corners]
    ys = [p.y for p in corners]
    zs = [p.z for p in corners]

    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def aabb_overlap(a, b, clearance=0.0):
    """Check AABB overlap with optional clearance."""
    (ax0, ax1), (ay0, ay1), (az0, az1) = a
    (bx0, bx1), (by0, by1), (bz0, bz1) = b

    ax0 -= clearance; ay0 -= clearance; az0 -= clearance
    ax1 += clearance; ay1 += clearance; az1 += clearance

    bx0 -= clearance; by0 -= clearance; bz0 -= clearance
    bx1 += clearance; by1 += clearance; bz1 += clearance

    return (
        ax0 <= bx1 and ax1 >= bx0 and
        ay0 <= by1 and ay1 >= by0 and
        az0 <= bz1 and az1 >= bz0
    )


# ============================================================
# STICK CLASS
# ============================================================

class Stick:
    DEFAULT_LENGTH = 100
    DEFAULT_SIZE   = 5

    def __init__(self, axis, length=None, width=None, depth=None):
        self.axis = axis
        self.length = length or Stick.DEFAULT_LENGTH
        self.width  = width  or Stick.DEFAULT_SIZE
        self.depth  = depth  or Stick.DEFAULT_SIZE
        self.frame  = self.compute_frame()

    def compute_frame(self):
        x = self.axis.direction.unitized()
        y = _stable_perp(x)
        z = x.cross(y).unitized()
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# ============================================================
# BRANCHING MODULE  (collision-safe L-system)
# ============================================================

class BranchingModule:

    def __init__(self, root_stick, stick_length, width, depth, offset01):
        self.sticks = [root_stick]
        self.len = stick_length
        self.w   = width
        self.d   = depth
        self.offset01 = float(offset01)

    # --- Build a child sticking out of a parent face ---
    def grow_once(self, face_index, angle_deg):
        parent = self.sticks[-1]
        pf = parent.frame

        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        fi = int(face_index) % 4
        if fi == 0:      # +Y
            n = pf.yaxis.unitized()
            half_p = self.w * 0.5
            half_c = self.w * 0.5
        elif fi == 2:    # -Y
            n = (-pf.yaxis).unitized()
            half_p = self.w * 0.5
            half_c = self.w * 0.5
        elif fi == 1:    # +Z
            n = pf.zaxis.unitized()
            half_p = self.d * 0.5
            half_c = self.d * 0.5
        else:            # -Z
            n = (-pf.zaxis).unitized()
            half_p = self.d * 0.5
            half_c = self.d * 0.5

        parent_face = axis_pt + n * half_p
        child_center = parent_face + n * half_c

        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = _stable_perp(n)
        tangent_proj.unitize()

        theta = math.radians(angle_deg)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)

        dproj = d_raw - n * d_raw.dot(n)
        if dproj.length < 1e-6:
            dproj = tangent_proj
        dproj.unitize()

        x = dproj
        y = n
        z = x.cross(y).unitized()
        frame = Frame(child_center, x, y)

        half = self.len * 0.5
        start = child_center - x * half
        end   = child_center + x * half
        axis = Line(start, end)

        child = Stick(axis, self.len, self.w, self.d)
        child.frame = frame
        self.sticks.append(child)

    def grow_chain(self, steps, face_index, angle_deg):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index, angle_deg)


# ============================================================
# BRIDGING MODULE (GrowTowards)
# ============================================================

class GrowTowards:

    def __init__(self, f0, f1, face_root, face_tgt,
                 offset_root, offset_tgt, length, width, depth):

        self.len = length
        self.w   = width
        self.d   = depth

        self.sticks = []

        c0, n0 = self.face_center(f0, face_root, offset_root)
        c1, n1 = self.face_center(f1, face_tgt,  offset_tgt)

        plane0 = Plane(c0, n0)
        plane1 = Plane(c1, n1)
        line = plane0.intersection_with_plane(plane1)
        if line:
            v = line.direction
            t = Vector.from_start_end(c0, line.start)
            s = t.dot(v)
            joint = line.start + v * s
        else:
            joint = Point(
                0.5 * (c0.x + c1.x),
                0.5 * (c0.y + c1.y),
                0.5 * (c0.z + c1.z),
            )

        self.sticks.append(self.build_child(c0, n0, joint))
        self.sticks.append(self.build_child(c1, n1, joint))

    # compute child center from face normal
    def face_center(self, fr, fi, off):
        fi = int(fi) % 4
        axis_pt = fr.point + fr.xaxis * float(off)

        if fi == 0:
            n = fr.yaxis.unitized()
            half = self.w * 0.5
        elif fi == 2:
            n = (-fr.yaxis).unitized()
            half = self.w * 0.5
        elif fi == 1:
            n = fr.zaxis.unitized()
            half = self.d * 0.5
        else:
            n = (-fr.zaxis).unitized()
            half = self.d * 0.5

        parent_face = axis_pt + n * half
        child_center = parent_face + n * half
        return child_center, n

    def build_child(self, center, n, joint):
        v = Vector.from_start_end(center, joint)
        vproj = v - n * v.dot(n)
        if vproj.length < 1e-6:
            vproj = _stable_perp(n)
        vproj.unitize()

        x = vproj
        y = n
        z = x.cross(y).unitized()
        frame = Frame(center, x, y)

        half = self.len * 0.5
        start = center - x * half
        end   = center + x * half
        axis = Line(start, end)

        s = Stick(axis, self.len, self.w, self.d)
        s.frame = frame
        return s


# ============================================================
# ROOTFRAMES ENGINE
# ============================================================

class RootFrames:

    def __init__(
        self,
        surface=None,
        curve=None,
        height_subdiv=5,
        point_density=10,
        twist_angle=0.0,
        stick_length=100,
        stick_width=5,
        stick_depth=5
    ):
        self.surface = surface
        self.curve   = curve

        self.height_subdiv = int(height_subdiv)
        self.point_density = int(point_density)
        self.twist_angle   = float(twist_angle)

        self.len = float(stick_length)
        self.w   = float(stick_width)
        self.d   = float(stick_depth)

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []
        self.sticks = []
        self.collisions = []

        self._rg_face = None
        self._uv = []
        self._rg_curve = None
        self._ts = []

    # ----------------------------------------------------------
    # BLOCK 1 — Sampling
    # ----------------------------------------------------------

    def surface_to_points(self):
        pts = []
        self._uv = []
        self._ts = []
        self._rg_face = None
        self._rg_curve = None

        # Curve mode
        if self.curve is not None and self.surface is None:
            crv = self.curve
            self._rg_curve = crv
            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            count = max(1, self.point_density * self.height_subdiv)
            for _ in range(count):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._ts.append(t)

        # Surface/Brep mode
        else:
            brep = self.surface.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face
            dom_u = face.Domain(0)
            dom_v = face.Domain(1)

            count = max(1, self.point_density * self.height_subdiv)
            for _ in range(count):
                u = random.uniform(dom_u.T0, dom_u.T1)
                v = random.uniform(dom_v.T0, dom_v.T1)
                srf = face.PointAt(u, v)
                pts.append(srf)
                self._uv.append((u, v))

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------
    # BLOCK 2 — 3D Frames
    # ----------------------------------------------------------

    def points_to_frames(self, rot_tan=0, rot_norm=0):
        frames = []

        # Curve frames
        if self._rg_curve is not None and self._ts:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._ts):
                plane = crv.FrameAt(t)  # returns only plane
                xaxis = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z).unitized()
                yaxis = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z).unitized()
                if yaxis.length < 1e-6:
                    yaxis = _stable_perp(xaxis)
                f = Frame(pt, xaxis, yaxis)

                if rot_tan:
                    f.transform(Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), pt))
                if rot_norm:
                    f.transform(Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), pt))

                frames.append(f)

        # Surface frames
        elif self._rg_face is not None and self._uv:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv):
                ok, plane = face.FrameAt(u, v)
                if not ok:
                    f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                else:
                    xaxis = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    yaxis = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    xaxis.unitize()
                    if yaxis.length < 1e-6:
                        yaxis = _stable_perp(xaxis)
                    else:
                        yaxis.unitize()
                    f = Frame(pt, xaxis, yaxis)

                if rot_tan:
                    f.transform(Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), pt))
                if rot_norm:
                    f.transform(Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), pt))

                frames.append(f)

        self.frames = frames
        return frames

    # ----------------------------------------------------------
    # BLOCK 3 — Edge Vectors
    # ----------------------------------------------------------

    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)
        if N < 2:
            return [], []

        edges = set()
        for i in range(N):
            p = pts[i]
            best = 1e9
            jb = None
            for j in range(N):
                if i == j:
                    continue
                d = p.distance_to_point(pts[j])
                if d < best:
                    best, jb = d, j
            edges.add(tuple(sorted((i, jb))))

        edges = [(i, j) for (i, j) in edges]
        self.edges = edges

        edge_frames = []
        edge_vectors = []

        for i, j in edges:
            f = self.frames[i]
            p0, p1 = f.point, self.frames[j].point
            v = Vector.from_start_end(p0, p1)

            z = f.zaxis.unitized()
            x = v - z * v.dot(z)
            if x.length < 1e-6:
                x = f.xaxis.copy()
            x.unitize()
            y = z.cross(x).unitized()
            edge_frames.append(Frame(p0, x, y))
            edge_vectors.append(x)

        self.edge_frames = edge_frames
        self.edge_vectors = edge_vectors
        return edge_frames, edge_vectors

    # ----------------------------------------------------------
    # BLOCK 4 — Growth
    # ----------------------------------------------------------

    def grow_sticks(self, mode, face_index, angle, offset01, steps, bridge_index):
        sticks_out = []

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.len)
            s = Stick(axis, self.len, self.w, self.d)
            roots.append(s)
            sticks_out.append(s)

        # Branching mode
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(r, self.len, self.w, self.d, offset01)
                mod.grow_chain(steps, face_index, angle)
                sticks_out.extend(mod.sticks[1:])
            self.sticks = sticks_out
            return sticks_out

        # Bridging mode
        if mode == "bridge":
            off = offset01 * self.len
            for (i, j) in self.edges:
                if bridge_index is not None:
                    bi = int(bridge_index)
                    if i != bi and j != bi:
                        continue

                f0 = self.frames[i]
                f1 = self.frames[j]

                grow = GrowTowards(
                    f0, f1,
                    face_index,          # root face
                    (face_index + 2) % 4,  # opposite face
                    off, off,
                    self.len, self.w, self.d
                )
                sticks_out.extend(grow.sticks)

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode: {}".format(mode))

    # ----------------------------------------------------------
    # Collision Detection
    # ----------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        n = len(self.sticks)
        flags = [False] * n
        aabbs = [stick_aabb(s) for s in self.sticks]

        for i in range(n):
            for j in range(i+1, n):
                if aabb_overlap(aabbs[i], aabbs[j], clearance):
                    flags[i] = True
                    flags[j] = True

        self.collisions = flags
        return flags

    # ----------------------------------------------------------
    # RUN
    # ----------------------------------------------------------

    def run(self, rot_tan=0, rot_norm=0,
            mode="branch", face_index=0, angle=0.0,
            offset01=1.0, steps=1, bridge_index=None,
            detect=False, clearance=0.0):

        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        sticks = self.grow_sticks(
            mode, face_index, angle,
            offset01, steps, bridge_index
        )

        if detect:
            self.detect_collisions(clearance)

        return sticks
