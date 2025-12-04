# ========================================================================
# RootFrames.py  —  Clean, Coherent, Rhino-Safe, No Parametric Frame Calls
# ========================================================================

import math
import random

from compas.geometry import (
    Point, Vector, Line, Frame, Box, Rotation, Transformation
)

# ------------------------------------------------------------------------
# UTILITIES
# ------------------------------------------------------------------------

def stable_perp(v):
    """Return a perpendicular vector that is stable for all directions."""
    z = Vector(0, 0, 1)
    y = Vector(0, 1, 0)
    up = z if abs(v.dot(z)) < 0.9 else y
    perp = up.cross(v)
    perp.unitize()
    return perp


def aabb_from_box(box):
    xs = [v.x for v in box.vertices]
    ys = [v.y for v in box.vertices]
    zs = [v.z for v in box.vertices]
    return (
        (min(xs), min(ys), min(zs)),
        (max(xs), max(ys), max(zs)),
    )


def aabb_overlap(a, b):
    (aminx, aminy, aminz), (amaxx, amaxy, amaxz) = a
    (bminx, bminy, bminz), (bmaxx, bmaxy, bmaxz) = b
    return (
        aminx <= bmaxx and amaxx >= bminx and
        aminy <= bmaxy and amaxy >= bminy and
        aminz <= bmaxz and amaxz >= bminz
    )


# ------------------------------------------------------------------------
# BASIC STICK
# ------------------------------------------------------------------------

class Stick:
    LENGTH = 100.0
    WIDTH  = 5.0
    DEPTH  = 5.0

    def __init__(self, axis, length=None, width=None, depth=None):
        self.axis   = axis
        self.length = length or Stick.LENGTH
        self.width  = width  or Stick.WIDTH
        self.depth  = depth  or Stick.DEPTH
        self.frame  = self.build_frame()

    def build_frame(self):
        x = self.axis.direction.unitized()
        y = stable_perp(x)
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return a Box oriented to the stick frame."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# ------------------------------------------------------------------------
# BRANCHING MODULE  —  Simple, Collision-Correct Child Construction
# ------------------------------------------------------------------------

class BranchingModule:

    def __init__(self, root, stick_length, width, depth, offset01):
        self.sticks = [root]
        self.len    = stick_length
        self.w      = width
        self.d      = depth
        self.offset = offset01

    def grow_once(self, face_index=0, angle=0):
        parent = self.sticks[-1]
        pf     = parent.frame

        # 1) Determine face normal
        fi = int(face_index) % 4
        if fi == 0:   n = pf.yaxis.unitized()
        elif fi == 2: n = (-pf.yaxis).unitized()
        elif fi == 1: n = pf.zaxis.unitized()
        else:         n = (-pf.zaxis).unitized()

        # 2) Parent face center
        t = max(0, min(1, self.offset))
        axis_pt = parent.axis.point_at(t)

        half_parent = (self.w if fi in (0, 2) else self.d) * 0.5
        half_child  = half_parent

        parent_face_center = axis_pt + n * half_parent
        child_center       = parent_face_center + n * half_child

        # 3) Tangent projection
        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = stable_perp(n)
        tangent_proj.unitize()

        # 4) Designer angle blend
        theta = math.radians(angle)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)

        # remove any normal component
        d = d_raw - n * d_raw.dot(n)
        if d.length < 1e-6:
            d = tangent_proj
        d.unitize()

        # 5) Build child
        half = self.len * 0.5
        start = child_center - d * half
        end   = child_center + d * half
        axis  = Line(start, end)

        child = Stick(axis, length=self.len, width=self.w, depth=self.d)
        self.sticks.append(child)

    def grow_chain(self, steps, face_index, angle):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index, angle)


# ------------------------------------------------------------------------
# GROW-TOWARDS (BRIDGING)
# ------------------------------------------------------------------------

class GrowTowards:

    def __init__(self, f0, f1, stick_length, width, depth, offset, face_index):
        self.len = stick_length
        self.w   = width
        self.d   = depth
        self.sticks = []

        c0, n0 = self.child_center(f0, offset, face_index)
        c1, n1 = self.child_center(f1, offset, (face_index + 2) % 4)

        joint = (c0 + c1) * 0.5

        self.sticks.append(self.build_child(c0, n0, joint))
        self.sticks.append(self.build_child(c1, n1, joint))

    def child_center(self, frame, offset, fi):
        axis_pt = frame.point + frame.xaxis * offset

        if fi == 0:   n = frame.yaxis.unitized(); half = self.w * 0.5
        elif fi == 2: n = (-frame.yaxis).unitized(); half = self.w * 0.5
        elif fi == 1: n = frame.zaxis.unitized(); half = self.d * 0.5
        else:         n = (-frame.zaxis).unitized(); half = self.d * 0.5

        return axis_pt + n * half * 2, n

    def build_child(self, center, n, joint):
        v = Vector.from_start_end(center, joint)
        v_proj = v - n * v.dot(n)
        if v_proj.length < 1e-6:
            v_proj = stable_perp(n)
        v_proj.unitize()

        half = self.len * 0.5
        start = center - v_proj * half
        end   = center + v_proj * half
        axis  = Line(start, end)

        return Stick(axis, length=self.len, width=self.w, depth=self.d)


# ------------------------------------------------------------------------
# ROOTFRAMES  —  CLEAN REWRITE
# ------------------------------------------------------------------------

class RootFrames:

    def __init__(self, surface=None, curve=None,
                 height_subdiv=5, point_density=10,
                 stick_length=100, stick_width=5, stick_depth=5):

        self.surface = surface
        self.curve   = curve

        self.height_subdiv = height_subdiv
        self.point_density = point_density

        self.len = stick_length
        self.w   = stick_width
        self.d   = stick_depth

        self.points = []
        self.frames = []
        self.edges  = []
        self.sticks = []

        self.collision_flags = []

    # ------------------------------------------------------------
    # 1) Sampling
    # ------------------------------------------------------------

    def surface_to_points(self):
        pts = []

        if self.curve and not self.surface:
            crv = self.curve
            dom = crv.Domain
            for _ in range(self.point_density * self.height_subdiv):
                t = random.uniform(dom.T0, dom.T1)
                p = crv.PointAt(t)
                pts.append(Point(p.X, p.Y, p.Z))

        else:
            brep = self.surface.ToBrep()
            face = brep.Faces[0]
            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(self.point_density * self.height_subdiv):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(Point(p.X, p.Y, p.Z))

        pts.sort(key=lambda p: p.z)
        self.points = pts
        return pts

    # ------------------------------------------------------------
    # 2) Build 3D frames with NO Rhino parameterization
    # ------------------------------------------------------------

    def points_to_frames(self):
        frames = []
        pts = self.points
        N = len(pts)
        if N < 2:
            self.frames = []
            return frames

        for i, p in enumerate(pts):
            j = min(i + 1, N - 1)
            t = Vector.from_start_end(p, pts[j])
            if t.length < 1e-6:
                t = Vector(1, 0, 0)
            t.unitize()
            y = stable_perp(t)
            frames.append(Frame(p, t, y))

        self.frames = frames
        return frames

    # ------------------------------------------------------------
    # 3) Build edges (nearest neighbor)
    # ------------------------------------------------------------

    def frames_to_edges(self):
        pts = self.points
        N = len(pts)
        if N < 2:
            self.edges = []
            return []

        edges = set()
        for i in range(N):
            pi = pts[i]
            dmin = 1e9
            jbest = None
            for j in range(N):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < dmin:
                    dmin = d
                    jbest = j
            if jbest is not None:
                edges.add(tuple(sorted((i, jbest))))

        self.edges = sorted(edges)
        return self.edges

    # ------------------------------------------------------------
    # 4) Growth
    # ------------------------------------------------------------

    def grow_sticks(self, mode, face_index, angle, offset01, steps, bridge_index):

        sticks = []

        # root sticks on each frame
        for f in self.frames:
            axis = Line(f.point, f.point + f.xaxis * self.len)
            sticks.append(Stick(axis, self.len, self.w, self.d))

        if mode == "branch":
            out = []
            for root in sticks:
                B = BranchingModule(root, self.len, self.w, self.d, offset01)
                B.grow_chain(steps, face_index, angle)
                out.extend(B.sticks)
            self.sticks = out
            return out

        if mode == "bridge":
            out = sticks[:]
            offset_abs = offset01 * self.len
            for (i, j) in self.edges:
                if bridge_index is not None:
                    bi = int(bridge_index)
                    if i != bi and j != bi:
                        continue
                G = GrowTowards(
                    self.frames[i], self.frames[j],
                    self.len, self.w, self.d,
                    offset_abs, face_index
                )
                out.extend(G.sticks)
            self.sticks = out
            return out

        raise Exception("Unknown mode")

    # ------------------------------------------------------------
    # 5) Collision detection
    # ------------------------------------------------------------

    def detect_collisions(self):
        boxes = [s.geometry for s in self.sticks]
        aabbs = [aabb_from_box(b) for b in boxes]

        flags = [False] * len(boxes)

        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                if aabb_overlap(aabbs[i], aabbs[j]):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------
    # RUN
    # ------------------------------------------------------------

    def run(self, mode="branch",
            face_index=0, angle=0,
            offset01=0.5, steps=1, bridge_index=None,
            detect_coll=False):

        self.surface_to_points()
        self.points_to_frames()
        self.frames_to_edges()

        sticks = self.grow_sticks(
            mode, face_index, angle,
            offset01, steps, bridge_index
        )

        if detect_coll:
            self.detect_collisions()

        return sticks
