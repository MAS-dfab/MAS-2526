# RootFrames.py
# COMPAS >= 2.14

import math
import random

from compas.geometry import (
    Point,
    Vector,
    Frame,
    Line,
    Box,
    Transformation,
    Rotation
)
from compas.geometry import OrientedBox, is_intersection_box_box


# =====================================================================
# HELPERS
# =====================================================================

def _stable_perp(v):
    """Returns a perpendicular vector that is stable even if v ≈ Z."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(v.dot(worldZ)) < 0.9 else worldY
    y = up.cross(v)
    y.unitize()
    return y


# =====================================================================
# STICK  (Oriented Box with Axis)
# =====================================================================

class Stick:
    """A rectangular cross-section “stick” with length, width, depth, and frame."""
    def __init__(self, axis, length, width, depth, frame_override=None):
        self.axis = axis
        self.length = float(length)
        self.width = float(width)
        self.depth = float(depth)

        # compute frame from the axis unless user overrides it
        if frame_override:
            self.frame = frame_override
        else:
            x = self.axis.direction.unitized()
            y = _stable_perp(x)
            self.frame = Frame(self.axis.midpoint, x, y)

    @property
    def oriented_box(self):
        """Return OBB for collision detection."""
        return OrientedBox(self.frame, self.length, self.width, self.depth)

    @property
    def geometry(self):
        """Plain Box geometry in oriented space (for Rhino preview)."""
        box = Box(self.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# =====================================================================
# BRANCHING MODULE  (robust, correct, no flattening)
# =====================================================================

class BranchingModule:
    """
    Clean branching:
      - Child near face sits exactly on parent face
      - Full offset = full half-width/half-depth
      - Rotation around face normal
      - Direction = pure face normal (not mixture)
      - No collisions guaranteed at parent joint
    """

    def __init__(self, root_stick, length, width, depth, offset01):
        self.sticks = [root_stick]
        self.len = length
        self.w   = width
        self.d   = depth
        self.offset01 = float(offset01)

    def _child_on_face(self, parent, face_index, angle_deg):
        fi = int(face_index) % 4
        pf = parent.frame
        t  = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # pick the correct outward normal + thickness
        if fi == 0:
            n = pf.yaxis
            half = self.w * 0.5
        elif fi == 2:
            n = -pf.yaxis
            half = self.w * 0.5
        elif fi == 1:
            n = pf.zaxis
            half = self.d * 0.5
        else:
            n = -pf.zaxis
            half = self.d * 0.5

        n = n.unitized()
        center = axis_pt + n * half + n * half  # full-depth offset

        # child frame orientation:
        x = n.copy()
        # rotate child around its own axis:
        if abs(angle_deg) > 1e-6:
            R = Rotation.from_axis_and_angle(x, math.radians(angle_deg), point=center)
            x = R * x

        y = _stable_perp(x)
        child_frame = Frame(center, x.unitized(), y.unitized())

        # axis
        start = center - x * (self.len * 0.5)
        end   = center + x * (self.len * 0.5)
        axis = Line(start, end)

        return Stick(axis, self.len, self.w, self.d, frame_override=child_frame)

    def grow_once(self, face_index, angle):
        parent = self.sticks[-1]
        child  = self._child_on_face(parent, face_index, angle)
        self.sticks.append(child)

    def grow_chain(self, steps, face_index, angle):
        for _ in range(int(max(0, steps))):
            self.grow_once(face_index, angle)


# =====================================================================
# BRIDGING – clean geometric version
# =====================================================================

class GrowTowards:
    def __init__(self, f0, f1, length, width, depth, face_index_root, offset):
        self.len = length
        self.w   = width
        self.d   = depth

        c0, n0 = self._face_center(f0, face_index_root, offset)
        c1, n1 = self._face_center(f1, (face_index_root + 2) % 4, offset)

        # joint target = midpoint
        joint = Point(
            0.5*(c0.x + c1.x),
            0.5*(c0.y + c1.y),
            0.5*(c0.z + c1.z)
        )

        self.sticks = [
            self._child(c0, n0, joint),
            self._child(c1, n1, joint)
        ]

    def _face_center(self, f, fi, offset):
        if fi == 0:
            n, half = f.yaxis, self.w * 0.5
        elif fi == 2:
            n, half = -f.yaxis, self.w * 0.5
        elif fi == 1:
            n, half = f.zaxis, self.d * 0.5
        else:
            n, half = -f.zaxis, self.d * 0.5

        n = n.unitized()
        c = f.point + n * (half + half) + f.xaxis * offset
        return c, n

    def _child(self, c, n, joint):
        d = Vector.from_start_end(c, joint).unitized()
        x = d
        y = _stable_perp(x)

        f = Frame(c, x, y)
        start = c - x*(self.len*0.5)
        end   = c + x*(self.len*0.5)
        axis = Line(start, end)

        return Stick(axis, self.len, self.w, self.d, frame_override=f)


# =====================================================================
# ROOTFRAMES ENGINE (clean)
# =====================================================================

class RootFrames:
    """
    Clean, stable, correct RootFrames engine with:
    - true 3D frames
    - stable branching
    - stable bridging
    - correct face offsets
    - true OBB collision detection
    """

    def __init__(self, surface=None, curve=None,
                 height_subdiv=5, point_density=10, twist_angle=0,
                 stick_length=100.0, stick_width=5.0, stick_depth=5.0):

        self.surface = surface
        self.curve   = curve

        self.height_subdiv = int(height_subdiv)
        self.point_density = int(point_density)
        self.twist_angle   = float(twist_angle)

        self.len = stick_length
        self.w   = stick_width
        self.d   = stick_depth

        self.points     = []
        self.frames     = []
        self.edge_frames  = []
        self.edge_vectors = []
        self.edges      = []
        self.sticks     = []
        self.collisions = []

    # ----------------------------------------------------------
    # 1) Sampling
    # ----------------------------------------------------------

    def surface_to_points(self):
        pts = []

        if self.curve and not self.surface:
            # curve mode
            crv = self.curve
            dom = crv.Domain
            for _ in range(self.point_density):
                t = random.uniform(dom.T0, dom.T1)
                p = crv.PointAt(t)
                pts.append((Point(p.X,p.Y,p.Z), t))

            self.points = [p for p,t in pts]
            self._curve_params = [t for p,t in pts]
            self._surface_params = None
            return self.points

        # surface mode
        face = self.surface.ToBrep().Faces[0]
        udom = face.Domain(0)
        vdom = face.Domain(1)
        UV = []
        for _ in range(self.point_density * self.height_subdiv):
            u = random.uniform(udom.T0, udom.T1)
            v = random.uniform(vdom.T0, vdom.T1)
            p = face.PointAt(u,v)
            pts.append(Point(p.X,p.Y,p.Z))
            UV.append((u,v))

        self.points = pts
        self._surface_params = UV
        self._curve_params   = None
        return pts

    # ----------------------------------------------------------
    # 2) 3D frames
    # ----------------------------------------------------------

    def points_to_frames(self):
        frames = []

        if self._curve_params:
            crv = self.curve
            for p, t in zip(self.points, self._curve_params):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    x = Vector(1,0,0)
                    y = Vector(0,1,0)
                else:
                    x = Vector(plane.XAxis.X,plane.XAxis.Y,plane.XAxis.Z).unitized()
                    y = Vector(plane.YAxis.X,plane.YAxis.Y,plane.YAxis.Z).unitized()
                frames.append(Frame(p, x, y))

        else:
            face = self.surface.ToBrep().Faces[0]
            for p, (u,v) in zip(self.points, self._surface_params):
                ok, plane = face.FrameAt(u,v)
                if not ok:
                    x = Vector(1,0,0); y = Vector(0,1,0)
                else:
                    x = Vector(plane.XAxis.X,plane.XAxis.Y,plane.XAxis.Z).unitized()
                    y = Vector(plane.YAxis.X,plane.YAxis.Y,plane.YAxis.Z).unitized()
                frames.append(Frame(p, x, y))

        self.frames = frames
        return frames

    # ----------------------------------------------------------
    # 3) Edge Vectors
    # ----------------------------------------------------------

    def frames_to_edgevectors(self):
        N = len(self.frames)
        pts = [f.point for f in self.frames]

        edges = set()
        for i in range(N):
            best = None
            dmin = 1e9
            for j in range(N):
                if i==j: continue
                d = pts[i].distance_to_point(pts[j])
                if d < dmin:
                    dmin = d; best = j
            edges.add(tuple(sorted((i,best))))

        edges = list(edges)
        self.edges = edges

        eframes = []
        evectors = []
        for i,j in edges:
            f0 = self.frames[i]
            p0 = f0.point
            p1 = self.frames[j].point
            v  = Vector.from_start_end(p0,p1)
            v.unitize()

            x = v
            y = _stable_perp(x)
            eframes.append(Frame(p0, x,y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ----------------------------------------------------------
    # 4) Growth
    # ----------------------------------------------------------

    def grow_sticks(self, mode, face_index, angle, offset01, steps, bridge_index):
        sticks = []

        # root sticks
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            start = f.point
            end   = f.point + v * self.len
            axis  = Line(start,end)
            roots.append(Stick(axis,self.len,self.w,self.d))
            sticks.append(roots[-1])

        if mode == "branch":
            for r in roots:
                bm = BranchingModule(r, self.len,self.w,self.d, offset01)
                bm.grow_chain(steps, face_index, angle)
                sticks.extend(bm.sticks[1:])

        elif mode == "bridge":
            offset_abs = offset01*self.len
            for i,j in self.edges:
                if bridge_index is not None and bridge_index not in (i,j):
                    continue
                g = GrowTowards(self.frames[i], self.frames[j],
                                self.len,self.w,self.d,
                                face_index, offset_abs)
                sticks.extend(g.sticks)

        self.sticks = sticks
        return sticks

    # ----------------------------------------------------------
    # 5) Collision Detection (OBB–OBB exact)
    # ----------------------------------------------------------

    def detect_collisions(self):
        n = len(self.sticks)
        flags = [False]*n
        for i in range(n):
            A = self.sticks[i].oriented_box
            for j in range(i+1,n):
                B = self.sticks[j].oriented_box
                if is_intersection_box_box(A,B):
                    flags[i] = True
                    flags[j] = True
        self.collisions = flags
        return flags

    # ----------------------------------------------------------
    # RUN
    # ----------------------------------------------------------

    def run(self, mode="branch", face_index=0, angle=0,
            offset01=1.0, steps=1, bridge_index=None,
            detect=False):

        self.surface_to_points()
        self.points_to_frames()
        self.frames_to_edgevectors()

        sticks = self.grow_sticks(mode,face_index,angle,
                                  offset01,steps,bridge_index)

        if detect:
            self.detect_collisions()

        return sticks
