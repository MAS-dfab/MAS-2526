# ======================================================================
# RootFrames.py   (compas >= 2.14.1)
# Fully corrected: 3D edge frames, true face offsets, clean branching.
# ======================================================================

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box, Plane,
    Rotation, Transformation, closest_point_on_line
)

# ======================================================================
# HELPERS
# ======================================================================

def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


# ======================================================================
# STICK CLASS
# ======================================================================

class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None, z_vector=None):
        """
        axis   : compas.geometry.Line (centerline)
        length : box length (local X)
        width  : box width  (local Y)
        depth  : box depth  (local Z)
        """
        self.axis = axis
        self.length = length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.frame = self.compute_frame(z_vector=z_vector)

    def compute_frame(self, z_vector=None):
        x = self.axis.direction.unitized()

        if z_vector:
            z = z_vector.unitized()
            y = z.cross(x).unitized()
        else:
            y = _stable_perp(x)
            z = x.cross(y).unitized()

        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# ======================================================================
# BRANCHING MODULE (corrected 3D + no collisions)
# ======================================================================

class BranchingModule:
    """
    Correct L-system growth:
    - child grows from a parent face
    - offset = true thickness (full width/depth)
    - normal is 3D-correct
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

    # ------------------------------------------------------------------

    def _face_frame(self, parent, face_index):
        """
        Compute the correct face frame on the parent stick.
        Full face offset is applied (true non-collision).
        """
        f = parent.frame.copy()
        fi = int(face_index) % 4

        # Rotate frame around parent’s local X-axis
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        # Apply TRUE face displacement
        if fi in (0, 2):   # ±Y faces → full width
            f.point += f.yaxis * (self.width)
            normal = f.yaxis.unitized()
        else:              # ±Z faces → full depth
            f.point += f.zaxis * (self.depth)
            normal = f.zaxis.unitized()

        return f, normal

    # ------------------------------------------------------------------

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        base_frame, normal = self._face_frame(parent, face_index)

        # Designer rotation around face normal
        if stick_angle:
            R = Rotation.from_axis_and_angle(normal, math.radians(stick_angle), point=base_frame.point)
            base_frame.transform(R)

        # Grow along the true 3D face normal
        axis = Line(base_frame.point, base_frame.point + normal * self.stick_length)

        child = Stick(axis,
                      length=self.stick_length,
                      width=self.width,
                      depth=self.depth)

        self.sticks.append(child)

    # ------------------------------------------------------------------

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# ======================================================================
# GROW TOWARDS (BRIDGING) — corrected 3D face normals + true offsets
# ======================================================================

class GrowTowards:

    def __init__(
        self,
        root_frame,
        target_frame,
        offset_root_child=0.0,
        offset_target_child=0.0,
        stick_length=None,
        width=None,
        depth=None,
        face_index_root=0,
        face_index_target=None
    ):
        self.len = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_frame = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child = offset_root_child
        self.offset_target_child = offset_target_child

        self.face_index_root = int(face_index_root) % 4
        self.face_index_target = (
            (self.face_index_root + 2) % 4 if face_index_target is None
            else int(face_index_target) % 4
        )

        self.sticks = []

        # Build child frames
        self.f_root = self._face_child_frame(self.root_frame, self.face_index_root, self.offset_root_child)
        self.f_tgt  = self._face_child_frame(self.target_frame, self.face_index_target, self.offset_target_child)

        # Intersection logic
        plane0 = Plane.from_frame(self.f_root)
        plane1 = Plane.from_frame(self.f_tgt)
        joint = plane0.intersection_with_plane(plane1)

        if joint:
            jp = Point(*closest_point_on_line(self.f_root.point, joint))
        else:
            jp = Point(
                0.5 * (self.f_root.point.x + self.f_tgt.point.x),
                0.5 * (self.f_root.point.y + self.f_tgt.point.y),
                0.5 * (self.f_root.point.z + self.f_tgt.point.z),
            )

        self.root_stick  = self._build(self.f_root, jp)
        self.target_stick = self._build(self.f_tgt, jp)

        self.sticks.extend([self.root_stick, self.target_stick])

    # ------------------------------------------------------------------

    def _face_child_frame(self, f, fi, offset):
        f = f.copy()

        # Slide along X-axis
        f.point = f.point + f.xaxis * float(offset)

        # Rotate to face
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        # Apply true face offset
        if fi in (0, 2):
            f.point += f.yaxis * (self.width)
        else:
            f.point += f.zaxis * (self.depth)

        return f

    # ------------------------------------------------------------------

    def _build(self, frame, joint):
        origin = frame.point
        direction = Vector.from_start_end(origin, joint)
        if direction.length < 1e-6:
            direction = frame.xaxis.copy()
        direction.unitize()

        axis = Line(origin, origin + direction * self.len)
        return Stick(axis, length=self.len, width=self.width, depth=self.depth)

    # ------------------------------------------------------------------

    def visualize(self):
        return [s.geometry for s in self.sticks]


# ======================================================================
# ROOTFRAMES ENGINE — 3D-correct sampling & vector logic
# ======================================================================

class RootFrames:

    def __init__(
        self,
        surface=None, curve=None,
        height_subdiv=5, point_density=10, twist_angle=0,
        stick_length=None, stick_width=None, stick_depth=None
    ):
        self.surface_input = surface
        self.curve_input = curve

        self.height_subdiv = int(height_subdiv)
        self.point_density = int(point_density)
        self.twist_angle = float(twist_angle)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width  = stick_width  or Stick.WIDTH
        self.stick_depth  = stick_depth  or Stick.DEPTH

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []
        self.sticks = []

    # ==================================================================
    # BLOCK 1 — Point sampling
    # ==================================================================

    def surface_to_points(self):
        import Rhino.Geometry as rg

        pts = []

        # --- Curve mode extrude + twist --------------------------------
        if self.curve_input is not None and self.surface_input is None:
            surf = rg.Surface.CreateExtrusion(self.curve_input, rg.Vector3d(0,0,1)*10)
            brep = surf.ToBrep()
            face = brep.Faces[0]

            for k in range(self.height_subdiv):
                layer = []
                for _ in range(self.point_density):
                    u, v = random.random(), random.random()
                    p = face.PointAt(u, v)
                    layer.append(rg.Point3d(p.X, p.Y, k))

                # apply twist around centroid
                cx = sum(p.X for p in layer) / len(layer)
                cy = sum(p.Y for p in layer) / len(layer)
                ang = math.radians(self.twist_angle*k)

                for p in layer:
                    dx = p.X - cx
                    dy = p.Y - cy
                    X = cx + dx*math.cos(ang) - dy*math.sin(ang)
                    Y = cy + dx*math.sin(ang) + dy*math.cos(ang)
                    pts.append(rg.Point3d(X,Y,p.Z))

        # --- Surface mode ------------------------------------------------
        else:
            brep = self.surface_input.ToBrep()
            face = brep.Faces[0]
            udom, vdom = face.Domain(0), face.Domain(1)

            for _ in range(self.point_density * self.height_subdiv):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ==================================================================
    # BLOCK 2 — Frame generation
    # ==================================================================

    def points_to_frames(self, rot_tan=0, rot_norm=0):
        pts = self.points
        N = len(pts)
        frames = []

        if N < 2:
            self.frames = []
            return []

        Z = Vector(0,0,1)

        for i, p in enumerate(pts):
            prev = pts[max(i-1,0)]
            next = pts[min(i+1,N-1)]

            t = Vector(next.x-prev.x, next.y-prev.y, 0.0)
            if t.length < 1e-6:
                t = Vector(1,0,0)
            t.unitize()

            y = Z.cross(t)
            if y.length < 1e-6:
                y = Vector(0,1,0)
            y.unitize()

            f = Frame(p, t, y)

            if rot_tan:
                R = Rotation.from_axis_and_angle(t, math.radians(rot_tan), point=p)
                f.transform(R)

            if rot_norm:
                R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=p)
                f.transform(R)

            frames.append(f)

        self.frames = frames
        return frames

    # ==================================================================
    # BLOCK 3 — Correct 3D edge frames (fix for your planar collapse)
    # ==================================================================

    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)

        if N < 2:
            self.edge_frames, self.edge_vectors, self.edges = [], [], []
            return [], []

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
            edges.add(tuple(sorted((i,j_best))))

        edges = [(i,j) for (i,j) in edges if i<N and j<N]
        self.edges = edges

        eframes = []
        evectors = []

        for i,j in edges:
            f = self.frames[i]
            p0 = f.point
            p1 = self.frames[j].point

            # true 3D tangent
            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue

            # preserve surface normal!
            z = f.zaxis.unitized()

            x = v - z * v.dot(z)
            if x.length < 1e-6:
                x = f.xaxis.copy()
            x.unitize()

            y = z.cross(x).unitized()

            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames, self.edge_vectors = eframes, evectors
        return eframes, evectors

    # ==================================================================
    # BLOCK 4 — Growth core
    # ==================================================================

    def grow_sticks(self, mode="branch", face_index=0, angle=0.0, offset01=1.0, steps=1, bridge_index=None):
        mode = mode.strip().lower()
        sticks_out = []

        if not self.edge_frames:
            return sticks_out

        # Root sticks from edge frames
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v*self.stick_length)
            root = Stick(axis,
                         length=self.stick_length,
                         width=self.stick_width,
                         depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        # ------------------ BRANCHING ------------------
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(r,
                                      stick_length=self.stick_length,
                                      width=self.stick_width,
                                      depth=self.stick_depth)
                mod.grow_chain(steps=steps, face_index=face_index, stick_angle=angle)
                sticks_out.extend(mod.sticks[1:])  # remove duplicate root
            self.sticks = sticks_out
            return sticks_out

        # ------------------ BRIDGING -------------------
        if mode == "bridge":
            offset_abs = offset01 * self.stick_length

            for (i,j) in self.edges:
                if bridge_index is not None:
                    bi = int(bridge_index)
                    if i != bi and j != bi:
                        continue

                f0 = self.frames[i]
                f1 = self.frames[j]

                try:
                    grow = GrowTowards(
                        root_frame=f0,
                        target_frame=f1,
                        offset_root_child=offset_abs,
                        offset_target_child=offset_abs,
                        stick_length=self.stick_length,
                        width=self.stick_width,
                        depth=self.stick_depth,
                        face_index_root=face_index
                    )
                    sticks_out.extend(grow.sticks)

                except Exception as e:
                    print("GrowTowards error:", e)
                    continue

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode '{}'".format(mode))

    # ==================================================================
    # RUN
    # ==================================================================

    def run(self, rot_tan=0, rot_norm=0, mode="branch", face_index=0, angle=0.0, offset01=1.0, steps=1, bridge_index=None):
        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        return self.grow_sticks(
            mode=mode, face_index=face_index, angle=angle,
            offset01=offset01, steps=steps, bridge_index=bridge_index
        )

