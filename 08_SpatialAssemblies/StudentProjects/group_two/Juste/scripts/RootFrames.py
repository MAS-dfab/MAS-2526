# RootFrames.py
# Fully corrected 3D branching + true geometry frame inheritance
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

# ============================================================================
# HELPERS
# ============================================================================

def _stable_perp(xaxis):
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


def _distance_point_segment(pt, line):
    p0 = line.start
    p1 = line.end
    u = p1 - p0
    uu = u.dot(u)
    if uu < 1e-12:
        return pt.distance_to_point(p0)

    t = (pt - p0).dot(u) / uu
    t = max(0.0, min(1.0, t))
    cp = p0 + u * t
    return pt.distance_to_point(cp)


def _segment_distance(line1, line2):
    pts1 = [line1.start, (line1.start + line1.end) * 0.5, line1.end]
    pts2 = [line2.start, (line2.start + line2.end) * 0.5, line2.end]

    dmin = 1e9
    for p in pts1:
        for qline in [line2]:
            dmin = min(dmin, _distance_point_segment(p, qline))

    for p in pts2:
        for qline in [line1]:
            dmin = min(dmin, _distance_point_segment(p, qline))

    return dmin


# ============================================================================
# STICK  — stores both axis frame + geometry frame
# ============================================================================

class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None, geom_frame=None):
        self.axis = axis
        self.length = length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        # geometry frame that comes from surface/curve — CRITICAL FIX
        self.geom_frame = geom_frame

        # axis-aligned frame (not used for branching direction anymore)
        self.frame = self.compute_frame()

    def compute_frame(self):
        x = self.axis.direction.unitized()
        y = _stable_perp(x)
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# ============================================================================
# BRANCHING MODULE (TRUE 3D GROWTH)
# ============================================================================

class BranchingModule:

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    def _build_child(self, parent, face_index, stick_angle):
        # ------------------------------------------------------------------
        # CRITICAL FIX: use geometry frame, not stick frame
        # ------------------------------------------------------------------
        gf = parent.geom_frame  
        tangent = gf.xaxis.unitized()
        normal  = gf.zaxis.unitized()

        # blend tangent + normal in TRUE 3D
        theta = math.radians(stick_angle)
        direction = (normal * math.cos(theta)) + (tangent * math.sin(theta))
        direction.unitize()

        # compute axis based on direction
        half = self.stick_length * 0.5

        t = max(0.0, min(1.0, self.offset01))
        parent_pt = parent.axis.point_at(t)

        start = parent_pt - direction * half
        end   = parent_pt + direction * half
        axis  = Line(start, end)

        # child inherits original geometry frame
        child = Stick(
            axis,
            length=self.stick_length,
            width=self.width,
            depth=self.depth,
            geom_frame=parent.geom_frame,
        )
        return child

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(int(max(1, steps))):
            self.grow_once(face_index, stick_angle)


# ============================================================================
# ROOTFRAMES ENGINE — 3D, DOUBLE-CURVED SURFACE AWARE
# ============================================================================

class RootFrames:

    def __init__(self, surface=None, curve=None, point_density=10,
                 stick_length=None, stick_width=None, stick_depth=None):

        self.surface_input = surface
        self.curve_input = curve
        self.point_density = int(point_density)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width  = stick_width  or Stick.WIDTH
        self.stick_depth  = stick_depth  or Stick.DEPTH

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []
        self.sticks = []
        self.collision_flags = []

        self._rg_face = None
        self._rg_curve = None
        self._uv_params = []
        self._curve_t = []

    # ----------------------------------------------------------------------
    # 1. SAMPLE POINTS
    # ----------------------------------------------------------------------

    def _sample_points(self):
        import Rhino.Geometry as rg

        pts = []

        # Curve mode
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            for _ in range(self.point_density):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # Surface mode
        else:
            brep = self.surface_input.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(self.point_density):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. GET 3D FRAMES FROM GEOMETRY
    # ----------------------------------------------------------------------

    def _frames_from_geometry(self):
        import Rhino.Geometry as rg

        frames = []

        # Curve mode
        if self._rg_curve is not None:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z).unitized()
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z).unitized()
                else:
                    tangent = crv.TangentAt(t)
                    x = Vector(tangent.X, tangent.Y, tangent.Z).unitized()
                    y = _stable_perp(x)

                frames.append(Frame(pt, x, y))

        # Surface mode
        else:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)
                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z).unitized()
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z).unitized()
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)

                frames.append(Frame(pt, x, y))

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. BUILD EDGE VECTORS (3D)
    # ----------------------------------------------------------------------

    def _frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)
        if N < 2:
            self.edges = []
            return

        edges = set()
        for i in range(N):
            best = 1e9
            j_best = None
            for j in range(N):
                if i == j:
                    continue
                d = pts[i].distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j
            edges.add(tuple(sorted((i, j_best))))

        edges = [(i, j) for (i, j) in edges]
        self.edges = edges

        self.edge_frames = []
        self.edge_vectors = []

        for i, j in edges:
            p0, p1 = pts[i], pts[j]
            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue
            x = v.unitized()
            y = _stable_perp(x)
            self.edge_frames.append(Frame(p0, x, y))
            self.edge_vectors.append(x)

    # ----------------------------------------------------------------------
    # 4. GROW BRANCHES
    # ----------------------------------------------------------------------

    def _grow_sticks_branching(self, steps, face_index, stick_angle, offset01):
        output = []

        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)

            # root inherits TRUE geometry frame
            geom_frame = f  

            root = Stick(axis,
                         length=self.stick_length,
                         width=self.stick_width,
                         depth=self.stick_depth,
                         geom_frame=geom_frame)

            output.append(root)

            mod = BranchingModule(
                root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            mod.grow_chain(steps=steps, face_index=face_index, stick_angle=stick_angle)
            output.extend(mod.sticks[1:])

        self.sticks = output
        return output

    # ----------------------------------------------------------------------
    # 5. COLLISION
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        n = len(self.sticks)
        flags = [False] * n

        if n < 2:
            self.collision_flags = flags
            return flags

        thick = max(self.stick_width, self.stick_depth) + clearance

        for i in range(n):
            for j in range(i + 1, n):
                d = _segment_distance(self.sticks[i].axis, self.sticks[j].axis)
                if d < thick:
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ----------------------------------------------------------------------
    # 6. RUN ENGINE
    # ----------------------------------------------------------------------

    def run(self, steps=1, stick_angle=0.0, offset01=0.5, detect=False, clearance=0.0):
        self._sample_points()
        self._frames_from_geometry()
        self._frames_to_edgevectors()

        sticks = self._grow_sticks_branching(
            steps=steps,
            face_index=0,
            stick_angle=stick_angle,
            offset01=offset01,
        )

        if detect:
            self.detect_collisions(clearance)

        return sticks
