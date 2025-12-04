# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box, Plane,
    Rotation, Transformation, closest_point_on_line
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


# =============================================================================
# STICK
# =============================================================================

class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None, z_vector=None):
        """
        axis   : Line (centerline)
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


# =============================================================================
# BRANCHING MODULE (L-system chain, collision-safe)
# =============================================================================

class BranchingModule:
    """
    Branching system:
      - Each generation grows from the last stick
      - Child origin is offset by full width/depth → true face contact, no overlap
      - Child axis is a blend of face normal & parent tangent (controlled angle)
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)  # parameter 0–1 along parent axis

    # ------------------------------------------------------------------  
    # Compute child origin & normal on a given parent face
    # ------------------------------------------------------------------  

    def _child_origin_and_normal(self, parent, face_index):
        fi = int(face_index) % 4

        # 1. pick point on parent axis (centerline)
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # 2. local frame at that point (same orientation as parent)
        f = Frame(axis_pt, parent.frame.xaxis, parent.frame.yaxis)

        # 3. rotate around local X to select face (0,1,2,3)
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        # 4. choose face normal + thicknesses
        if fi in (0, 2):  # ±Y faces
            normal = f.yaxis.unitized()
            parent_half = self.width * 0.5
            child_half = self.width * 0.5
        else:            # ±Z faces
            normal = f.zaxis.unitized()
            parent_half = self.depth * 0.5
            child_half = self.depth * 0.5

        # 5. parent face center (volume boundary)
        parent_face = axis_pt + normal * parent_half

        # 6. child origin = parent face + half child thickness
        #    → child's near face plane coincides with parent face (no overlap)
        child_origin = parent_face + normal * child_half

        # keep original parent tangent for direction-blend
        parent_tangent = parent.frame.xaxis

        return child_origin, normal, parent_tangent

    # ------------------------------------------------------------------  
    # Grow one generation from the last stick
    # ------------------------------------------------------------------  

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]

        origin, normal, tangent = self._child_origin_and_normal(parent, face_index)

        # designer angle = blend between normal & tangent
        theta = math.radians(stick_angle)
        d = normal * math.cos(theta) + tangent * math.sin(theta)
        if d.length < 1e-6:
            d = normal
        d.unitize()

        axis = Line(origin, origin + d * self.stick_length)
        child = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        self.sticks.append(child)

    # ------------------------------------------------------------------  
    # N-step chain (L-system style)
    # ------------------------------------------------------------------  

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# GROW-TOWARDS (BRIDGE)
# =============================================================================

class GrowTowards:
    """
    Build two child sticks that start on chosen faces of root/target frames.
    Each child origin is offset by full width/depth from the centerline,
    then grown towards a joint point between their face planes.
    """

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

        self.offset_root_child = float(offset_root_child or 0.0)
        self.offset_target_child = float(offset_target_child or 0.0)

        self.face_index_root = int(face_index_root) % 4
        self.face_index_target = (
            (self.face_index_root + 2) % 4
            if face_index_target is None
            else int(face_index_target) % 4
        )

        self.sticks = []

        # child origins + normals at faces
        o0, n0 = self._child_origin_and_normal(
            self.root_frame, self.face_index_root, self.offset_root_child
        )
        o1, n1 = self._child_origin_and_normal(
            self.target_frame, self.face_index_target, self.offset_target_child
        )

        # intersection of face planes → joint point
        plane0 = Plane(o0, n0)
        plane1 = Plane(o1, n1)
        line = plane0.intersection_with_plane(plane1)

        if line:
            joint = Point(*closest_point_on_line(o0, line))
        else:
            # fallback midpoint
            joint = Point(
                0.5 * (o0.x + o1.x),
                0.5 * (o0.y + o1.y),
                0.5 * (o0.z + o1.z),
            )

        self.sticks.append(self._build_child(o0, joint))
        self.sticks.append(self._build_child(o1, joint))

    # ------------------------------------------------------------------  

    def _child_origin_and_normal(self, frame, face_index, offset_dist):
        fi = int(face_index) % 4

        # axis point along frame.xaxis
        axis_pt = frame.point + frame.xaxis * float(offset_dist)

        f = Frame(axis_pt, frame.xaxis, frame.yaxis)
        R = Rotation.from_axis_and_angle(f.xaxis, fi * math.pi / 2.0, point=f.point)
        f.transform(R)

        if fi in (0, 2):
            normal = f.yaxis.unitized()
            parent_half = self.width * 0.5
            child_half = self.width * 0.5
        else:
            normal = f.zaxis.unitized()
            parent_half = self.depth * 0.5
            child_half = self.depth * 0.5

        parent_face = axis_pt + normal * parent_half
        child_origin = parent_face + normal * child_half

        return child_origin, normal

    def _build_child(self, origin, joint):
        d = Vector.from_start_end(origin, joint)
        if d.length < 1e-6:
            d = Vector(1, 0, 0)
        d.unitize()
        axis = Line(origin, origin + d * self.len)
        return Stick(axis, length=self.len, width=self.width, depth=self.depth)

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# ROOTFRAMES ENGINE
# =============================================================================

class RootFrames:
    """
    Pipeline:
      1) Surface/Curve → points
      2) Points → frames
      3) Frames → edge frames + edge vectors
      4) Growth: branch (L-system) or bridge (GrowTowards)
    """

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
        self.stick_width = stick_width or Stick.WIDTH
        self.stick_depth = stick_depth or Stick.DEPTH

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []
        self.sticks = []

    # ------------------------------------------------------------------  
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------  

    def surface_to_points(self):
        import Rhino.Geometry as rg
        pts = []

        # Curve mode: extrude and sample
        if self.curve_input is not None and self.surface_input is None:
            surf = rg.Surface.CreateExtrusion(self.curve_input, rg.Vector3d(0, 0, 1) * 10.0)
            if not surf:
                raise Exception("Failed to extrude curve_input.")
            brep = surf.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("Extruded brep has no faces.")
            face = brep.Faces[0]

            for k in range(max(1, self.height_subdiv)):
                z = 10.0 * k / max(1, self.height_subdiv)
                twist = math.radians(self.twist_angle * k)

                layer = []
                for _ in range(max(1, self.point_density)):
                    u, v = random.random(), random.random()
                    p = face.PointAt(u, v)
                    layer.append(rg.Point3d(p.X, p.Y, z))

                cx = sum(p.X for p in layer) / len(layer)
                cy = sum(p.Y for p in layer) / len(layer)

                for p in layer:
                    dx, dy = p.X - cx, p.Y - cy
                    X = cx + dx * math.cos(twist) - dy * math.sin(twist)
                    Y = cy + dx * math.sin(twist) + dy * math.cos(twist)
                    pts.append(rg.Point3d(X, Y, p.Z))

        # Surface / Brep mode
        else:
            if self.surface_input is None:
                raise Exception("No surface_input for sampling.")

            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("surface_input.ToBrep() has no faces.")
            face = brep.Faces[0]

            udom = face.Domain(0)
            vdom = face.Domain(1)

            count = max(1, self.point_density * max(1, self.height_subdiv))
            for _ in range(count):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------  
    # BLOCK 2 – FRAMES
    # ------------------------------------------------------------------  

    def points_to_frames(self, rot_tan=0, rot_norm=0):
        pts = self.points
        N = len(pts)
        frames = []

        if N == 0:
            self.frames = []
            return []

        Z = Vector(0, 0, 1)

        for i, p in enumerate(pts):
            p_prev = pts[max(i - 1, 0)]
            p_next = pts[min(i + 1, N - 1)]

            t = Vector(p_next.x - p_prev.x, p_next.y - p_prev.y, 0.0)
            if t.length:
                t.unitize()
            else:
                t = Vector(1, 0, 0)

            y = Z.cross(t)
            if y.length:
                y.unitize()
            else:
                y = Vector(0, 1, 0)

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

    # ------------------------------------------------------------------  
    # BLOCK 3 – EDGE FRAMES / VECTORS
    # ------------------------------------------------------------------  

    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)

        if N < 2:
            self.edge_frames = []
            self.edge_vectors = []
            self.edges = []
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

            z = f.zaxis.unitized()
            x = v - z * v.dot(z)
            if x.length < 1e-6:
                x = f.xaxis.copy()
            x.unitize()
            y = z.cross(x).unitized()

            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------------  
    # BLOCK 4 – GROWTH
    # ------------------------------------------------------------------  

    def grow_sticks(
        self,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None
    ):
        """
        mode: 'branch' or 'bridge'
        face_index: which face to grow from (0–3)
        angle: blend angle (deg) between face normal and tangent (branch)
        offset01: [0,1] param along axis (branch) or fraction of stick length (bridge)
        steps: N-step chain in branch mode
        bridge_index: if set, only edges touching this frame index are bridged
        """
        mode = str(mode).strip().lower()
        sticks_out = []

        if not self.edge_frames:
            return sticks_out

        # Root sticks
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
            )
            roots.append(root)
            sticks_out.append(root)

        # ---------------- BRANCH MODE ----------------
        if mode == "branch":
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
                    stick_angle=angle,
                )
                sticks_out.extend(mod.sticks[1:])  # skip duplicate root

            self.sticks = sticks_out
            return sticks_out

        # ---------------- BRIDGE MODE ----------------
        if mode == "bridge":
            offset_abs = offset01 * self.stick_length

            for (i, j) in self.edges:
                if bridge_index is not None:
                    bi = int(bridge_index)
                    if i != bi and j != bi:
                        continue

                if i >= len(self.frames) or j >= len(self.frames):
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
                        face_index_root=face_index,
                    )
                    sticks_out.extend(grow.sticks)
                except Exception as e:
                    print("GrowTowards failed on edge ({}, {}): {}".format(i, j, e))
                    continue

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode: {}".format(mode))

    # ------------------------------------------------------------------  
    # RUN
    # ------------------------------------------------------------------  

    def run(
        self,
        rot_tan=0,
        rot_norm=0,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None
    ):
        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        return self.grow_sticks(
            mode=mode,
            face_index=face_index,
            angle=angle,
            offset01=offset01,
            steps=steps,
            bridge_index=bridge_index,
        )
