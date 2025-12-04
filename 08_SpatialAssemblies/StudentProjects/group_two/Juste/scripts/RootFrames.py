# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box,
    Plane, Rotation, Transformation, closest_point_on_line
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
    """Sampled segment–segment distance (good enough for collision hints)."""
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
        axis   : Line (centerline)
        length : box length (local X)
        width  : box width  (local Y)
        depth  : box depth  (local Z)
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
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# =============================================================================
# BRANCHING MODULE  (L-system style, collision-safe contact)
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

        # pick face normal & thickness
        if fi == 0:          # +Y
            n = pf.yaxis.unitized()
            half_parent = self.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 2:        # -Y
            n = (-pf.yaxis).unitized()
            half_parent = self.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 1:        # +Z
            n = pf.zaxis.unitized()
            half_parent = self.depth * 0.5
            half_child  = self.depth * 0.5
        else:                # -Z
            n = (-pf.zaxis).unitized()
            half_parent = self.depth * 0.5
            half_child  = self.depth * 0.5

        # parent face center (outer skin)
        parent_face_center = axis_pt + n * half_parent
        # child center so its near face sits on parent face
        child_center = parent_face_center + n * half_child

        # tangent direction projected off the normal
        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = _stable_perp(n)
        tangent_proj.unitize()

        # blend normal & tangent with designer angle
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tangent_proj * math.sin(theta)
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
        end   = child_center + x * half_len
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
# GROW-TOWARDS (BRIDGING between two frames)
# =============================================================================

class GrowTowards:
    """
    Build two child sticks starting on chosen faces of root/target frames.
    Near faces lie on the parent faces; axes aim roughly toward a joint point
    between the two face planes.
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
        self.len   = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_frame   = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child   = float(offset_root_child or 0.0)
        self.offset_target_child = float(offset_target_child or 0.0)

        self.face_index_root   = int(face_index_root) % 4
        self.face_index_target = (
            (self.face_index_root + 2) % 4
            if face_index_target is None
            else int(face_index_target) % 4
        )

        self.sticks = []

        c0, n0 = self._child_center_and_normal(
            self.root_frame, self.face_index_root,   self.offset_root_child
        )
        c1, n1 = self._child_center_and_normal(
            self.target_frame, self.face_index_target, self.offset_target_child
        )

        plane0 = Plane(c0, n0)
        plane1 = Plane(c1, n1)
        line = plane0.intersection_with_plane(plane1)

        if line:
            joint = Point(*closest_point_on_line(c0, line))
        else:
            joint = Point(
                0.5 * (c0.x + c1.x),
                0.5 * (c0.y + c1.y),
                0.5 * (c0.z + c1.z),
            )

        self.sticks.append(self._build_child(c0, n0, joint))
        self.sticks.append(self._build_child(c1, n1, joint))

    def _child_center_and_normal(self, frame, face_index, offset_dist):
        fi = int(face_index) % 4

        axis_pt = frame.point + frame.xaxis * float(offset_dist)

        if fi == 0:          # +Y
            n = frame.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:        # -Y
            n = (-frame.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:        # +Z
            n = frame.zaxis.unitized()
            half = self.depth * 0.5
        else:                # -Z
            n = (-frame.zaxis).unitized()
            half = self.depth * 0.5

        parent_face_center = axis_pt + n * half
        child_center = parent_face_center + n * half
        return child_center, n

    def _build_child(self, center, n, joint):
        v = Vector.from_start_end(center, joint)
        v_proj = v - n * v.dot(n)
        if v_proj.length < 1e-6:
            v_proj = _stable_perp(n)
        v_proj.unitize()

        x = v_proj
        y = n
        z = x.cross(y).unitized()

        half_len = self.len * 0.5
        start = center - x * half_len
        end   = center + x * half_len
        axis = Line(start, end)

        s = Stick(axis, length=self.len, width=self.width, depth=self.depth)
        s.frame = Frame(center, x, y)
        return s

    def visualize(self):
        return [s.geometry for s in self.sticks]


# =============================================================================
# ROOTFRAMES ENGINE
# =============================================================================

class RootFrames:
    """
    Pipeline:
      1) Surface/Curve → points
      2) Points → 3D frames (true surface/curve frames)
      3) Frames → edge frames + edge vectors
      4) Growth: branch (L-system) or bridge (GrowTowards)
      5) Optional: collision detection for resulting sticks
    """

    def __init__(
        self,
        surface=None, curve=None,
        height_subdiv=5, point_density=10, twist_angle=0,
        stick_length=None, stick_width=None, stick_depth=None
    ):
        self.surface_input = surface
        self.curve_input   = curve

        self.height_subdiv = int(height_subdiv)
        self.point_density = int(point_density)
        self.twist_angle   = float(twist_angle)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width  = stick_width  or Stick.WIDTH
        self.stick_depth  = stick_depth  or Stick.DEPTH

        self.points       = []
        self.frames       = []
        self.edge_frames  = []
        self.edge_vectors = []
        self.edges        = []
        self.sticks       = []
        self.collisions   = []

        # param storage for proper 3D frames
        self._rg_face     = None
        self._uv_params   = []
        self._rg_curve    = None
        self._curve_t     = []

    # ------------------------------------------------------------------  
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------  

    def surface_to_points(self):
        import Rhino.Geometry as rg
        pts = []

        self._uv_params = []
        self._curve_t   = []
        self._rg_face   = None
        self._rg_curve  = None

        # --- Curve mode: use true curve frames later -------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv
            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            count = max(1, self.point_density * max(1, self.height_subdiv))
            for k in range(count):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # --- Surface/Brep mode: store (u,v) for each sample ------------
        else:
            if self.surface_input is None:
                raise Exception("No surface_input for sampling.")

            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("surface_input.ToBrep() has no faces.")
            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            count = max(1, self.point_density * max(1, self.height_subdiv))
            for _ in range(count):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------  
    # BLOCK 2 – 3D FRAMES
    # ------------------------------------------------------------------  

# --------------------------------------------------------------
# BLOCK 2 — POINTS → FRAMES (correct 3D frames)
# --------------------------------------------------------------
    def points_to_frames(self, rot_tan=0, rot_norm=0):
        pts = self.points
        if not pts:
            self.frames = []
            return []

        frames = []

        # If we sampled from a BrepFace, grab its surface
        surface = None
        if self.surface_input:
            brep = self.surface_input.ToBrep()
            surface = brep.Faces[0].UnderlyingSurface()

        for p in pts:
            if surface:
                # Convert to UV space first
                success, uv = surface.ClosestPoint(rg.Point3d(p.x, p.y, p.z))
                if not success:
                    # fallback frame
                    frames.append(Frame(p, Vector(1,0,0), Vector(0,1,0)))
                    continue

                # Evaluate first derivatives
                ok, point3d, du, dv = surface.Evaluate(uv[0], uv[1], 1)
                if not ok:
                    frames.append(Frame(p, Vector(1,0,0), Vector(0,1,0)))
                    continue

                tangent_u = Vector(du.X, du.Y, du.Z)
                tangent_v = Vector(dv.X, dv.Y, dv.Z)

                if tangent_u.length < 1e-6:
                    tangent_u = Vector(1,0,0)
                else:
                    tangent_u.unitize()

                normal = tangent_u.cross(tangent_v)
                if normal.length < 1e-6:
                    normal = Vector(0,0,1)
                else:
                    normal.unitize()

                f = Frame(p, tangent_u, normal)

            else:
                # Curve-based fallback frame (previous version)
                f = Frame(p, Vector(1,0,0), Vector(0,1,0))

            # Apply frame rotations
            if rot_tan:
                R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=p)
                f.transform(R)

            if rot_norm:
                R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=p)
                f.transform(R)

            frames.append(f)

        self.frames = frames
        return frames


    # ------------------------------------------------------------------  
    # BLOCK 3 – EDGE FRAMES & VECTORS
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
        face_index: which face to grow/bridge from (0–3)
        angle: branch angle (deg)
        offset01: [0,1] param along axis for branching, scaled length for bridging
        steps: N-step chain in branch mode
        bridge_index: if set, only edges touching this frame index are bridged
        """
        mode = str(mode).strip().lower()
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            return sticks_out

        # root sticks
        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(axis,
                         length=self.stick_length,
                         width=self.stick_width,
                         depth=self.stick_depth)
            roots.append(root)
            sticks_out.append(root)

        # ---- BRANCH MODE ----------------------------------------------
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(
                    r,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth,
                    offset01=offset01
                )
                mod.grow_chain(
                    steps=steps,
                    face_index=face_index,
                    stick_angle=angle
                )
                sticks_out.extend(mod.sticks[1:])  # skip duplicated root
            self.sticks = sticks_out
            return sticks_out

        # ---- BRIDGE MODE ----------------------------------------------
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
                        face_index_root=face_index
                    )
                    sticks_out.extend(grow.sticks)
                except Exception as e:
                    print("GrowTowards failed on edge ({}, {}): {}".format(i, j, e))
                    continue

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode: {}".format(mode))

    # ------------------------------------------------------------------  
    # COLLISION DETECTION
    # ------------------------------------------------------------------  

    def detect_collisions(self, clearance=0.0):
        """
        Mark sticks that collide (approximately) based on centerline distances.
        clearance: extra distance added to thickness before flagging collisions.
        """
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collisions = flags
            return flags

        # thickness ~ max cross-section dimension
        base_thick = max(self.stick_width, self.stick_depth) + float(clearance)

        for i in range(n):
            li = self.sticks[i].axis
            for j in range(i + 1, n):
                lj = self.sticks[j].axis
                d = _segment_distance(li, lj)
                if d < base_thick:
                    flags[i] = True
                    flags[j] = True

        self.collisions = flags
        return flags

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
        bridge_index=None,
        detect_collisions=False,
        clearance=0.0
    ):
        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        sticks = self.grow_sticks(
            mode=mode,
            face_index=face_index,
            angle=angle,
            offset01=offset01,
            steps=steps,
            bridge_index=bridge_index
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)

        return sticks
