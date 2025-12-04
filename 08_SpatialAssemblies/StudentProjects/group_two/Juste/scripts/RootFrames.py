# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box,
    Plane, Rotation, Transformation,
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
    """Distance from a point to a line segment (in 3D)."""
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


def _segment_distance(line1, line2, samples=7):
    """
    Sample-based segment–segment distance.
    More samples => more robust 'OBB-lite' collision check.
    """
    pts1 = []
    pts2 = []

    for i in range(samples):
        t = float(i) / (samples - 1)
        p0 = line1.start + (line1.end - line1.start) * t
        p1 = line2.start + (line2.end - line2.start) * t
        pts1.append(p0)
        pts2.append(p1)

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
# BRANCHING MODULE (face-contact chain)
# =============================================================================

class BranchingModule:
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies exactly on a parent face (no overlap).
      - Child axis is a blend of parent tangent and face normal.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    # ------------------ core face-based child builder ------------------ #

    def _build_child_from_face(self, parent, face_index, stick_angle):
        """
        parent     : Stick
        face_index : 0 = +Y, 1 = +Z, 2 = -Y, 3 = -Z (in parent.frame space)
        stick_angle: angle in degrees, 0 = pure normal, 90 = tangent
        """
        fi = int(face_index) % 4
        pf = parent.frame

        # position along parent axis: 0..1
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # pick face normal & thickness in that direction
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

        # strict face contact:
        # parent outer face center
        parent_face_center = axis_pt + n * half_parent
        # child center so its near face lies in the same plane
        child_center = parent_face_center + n * half_child

        # parent tangent (x) projected to be orthogonal to n
        tangent = pf.xaxis
        tproj = tangent - n * tangent.dot(n)
        if tproj.length < 1e-6:
            tproj = _stable_perp(n)
        tproj.unitize()

        # blend normal + tangent by angle (0 => normal, 90 => tangent)
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tproj * math.sin(theta)
        if d_raw.length < 1e-6:
            d_raw = n
        d_raw.unitize()

        # ensure child actually leaves the face (not going "into" parent)
        if d_raw.dot(n) <= 0:
            d_raw = -d_raw

        x = d_raw
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

    # --------------------- public API --------------------- #

    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# GROW-TOWARDS (bridging between two frames)
# =============================================================================

class GrowTowards:
    """
    Build two child sticks starting on faces of root/target frames.
    Near faces lie on the parent faces; axes aim toward a joint point
    between the two face planes. Used only when frames are non-coplanar.
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
        face_index_target=2
    ):
        self.len   = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.root_frame   = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child   = float(offset_root_child or 0.0)
        self.offset_target_child = float(offset_target_child or 0.0)

        self.face_index_root   = int(face_index_root) % 4
        self.face_index_target = int(face_index_target) % 4

        self.sticks = []

        # child centers + normals
        c0, n0 = self._child_center_and_normal(
            self.root_frame,   self.face_index_root,   self.offset_root_child
        )
        c1, n1 = self._child_center_and_normal(
            self.target_frame, self.face_index_target, self.offset_target_child
        )

        plane0 = Plane(c0, n0)
        plane1 = Plane(c1, n1)
        line = plane0.intersection_with_plane(plane1)

        if line:
            # joint is closest point on intersection line to root child center
            joint = line.closest_point(c0)
        else:
            # fallback: midpoint of centers
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
        if v.length < 1e-6:
            v = _stable_perp(n)
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
      1) Surface/Curve → sample points
      2) Points → 3D frames
      3) Frames → neighbour edges + edge frames
      4) Growth:
           - Branching (face-contact chains) from each edge frame
           - Bridging only between non-coplanar neighbours
      5) Optional : collision detection (OBB-lite via centerlines)
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
        self.surface_input = surface
        self.curve_input   = curve

        self.point_density = int(point_density)

        self.stick_length = stick_length or Stick.LENGTH
        self.stick_width  = stick_width  or Stick.WIDTH
        self.stick_depth  = stick_depth  or Stick.DEPTH

        self.points       = []
        self.frames       = []
        self.edge_frames  = []
        self.edge_vectors = []
        self.edges        = []
        self.sticks       = []
        self.collision_flags = []

        # storage for param coords
        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

    # ------------------------------------------------------------------  
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------  

    def surface_to_points(self):
        """Sample either a Rhino surface/Brep or a Rhino curve."""
        import Rhino.Geometry as rg

        pts = []

        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

        # --- Curve mode ------------------------------------------------
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

        # --- Surface/Brep mode -----------------------------------------
        else:
            if self.surface_input is None:
                raise Exception("No surface_input for sampling.")

            # normalize to BrepFace
            if isinstance(self.surface_input, rg.Brep):
                brep = self.surface_input
            else:
                brep = self.surface_input.ToBrep()

            if not brep or brep.Faces.Count == 0:
                raise Exception("surface_input has no faces.")
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

    # ------------------------------------------------------------------  
    # BLOCK 2 – 3D FRAMES
    # ------------------------------------------------------------------  

    def points_to_frames(self):
        """Assign a 3D frame to each sampled point."""
        import Rhino.Geometry as rg

        N = len(self.points)
        if N == 0:
            self.frames = []
            return []

        frames = []

        # --- Curve frames ----------------------------------------------
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                tangent = crv.TangentAt(t)
                tvec = Vector(tangent.X, tangent.Y, tangent.Z)
                if tvec.length < 1e-6:
                    tvec = Vector(1, 0, 0)
                else:
                    tvec.unitize()
                y = _stable_perp(tvec)
                f = Frame(pt, tvec, y)
                frames.append(f)

        # --- Surface frames --------------------------------------------
        elif self._rg_face is not None and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                # normal
                nvec = face.NormalAt(u, v)
                n = Vector(nvec.X, nvec.Y, nvec.Z)
                if n.length < 1e-6:
                    n = Vector(0, 0, 1)
                else:
                    n.unitize()

                # approximate tangents by small param shifts
                du = (face.Domain(0).T1 - face.Domain(0).T0) * 1e-3
                dv = (face.Domain(1).T1 - face.Domain(1).T0) * 1e-3

                pu = face.PointAt(u + du, v)
                pv = face.PointAt(u, v + dv)

                tu = Vector(pu.X - pt.x, pu.Y - pt.y, pu.Z - pt.z)
                tv = Vector(pv.X - pt.x, pv.Y - pt.y, pv.Z - pt.z)

                if tu.length < 1e-6:
                    tu = _stable_perp(n)
                else:
                    tu.unitize()
                if tv.length < 1e-6:
                    tv = n.cross(tu)
                    if tv.length < 1e-6:
                        tv = _stable_perp(tu)
                    else:
                        tv.unitize()

                x = tu
                y = tv
                z = x.cross(y).unitized()
                # re-orthogonalize y to z and x
                y = z.cross(x).unitized()

                f = Frame(pt, x, y)
                frames.append(f)

        else:
            # last-resort fallback
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                frames.append(f)

        self.frames = frames
        return frames

    # ------------------------------------------------------------------  
    # BLOCK 3 – EDGE FRAMES & VECTORS
    # ------------------------------------------------------------------  

    def frames_to_edgevectors(self):
        """Nearest-neighbour edges + edge frames (full 3D)."""
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
            p0 = self.frames[i].point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue
            v.unitize()

            x = v
            y = _stable_perp(x)
            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------------  
    # BLOCK 4 – GROWTH (branch + bridge)
    # ------------------------------------------------------------------  

    def grow_sticks(self, steps=1, stick_angle=0.0, offset01=0.5):
        """
        Create:
          - root sticks along edge frames
          - branching chains from each root
          - automatic bridging between non-coplanar neighbours
        """
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            return sticks_out

        # ---------- ROOT STICKS + BRANCHING ----------------------------

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            # root axis centered at frame point
            half_len = self.stick_length * 0.5
            start = f.point - v * half_len
            end   = f.point + v * half_len
            axis = Line(start, end)
            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth
            )
            roots.append(root)
            sticks_out.append(root)

        # branch chains from each root
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
                face_index=0,        # +Y face by default
                stick_angle=stick_angle
            )
            sticks_out.extend(mod.sticks[1:])  # skip duplicate root

        # ---------- BRIDGING (non-coplanar only) -----------------------

        angle_threshold_deg = 5.0
        height_threshold = self.stick_depth  # simple heuristic

        for (i, j) in self.edges:
            if i >= len(self.frames) or j >= len(self.frames):
                continue

            f0 = self.frames[i]
            f1 = self.frames[j]

            # test coplanarity via normals + height diff
            n0 = f0.zaxis.unitized()
            n1 = f1.zaxis.unitized()
            angle = math.degrees(math.acos(max(-1.0, min(1.0, n0.dot(n1)))))
            dz = abs(f0.point.z - f1.point.z)

            coplanar = (angle < angle_threshold_deg) and (dz < height_threshold)

            if coplanar:
                continue  # let branching handle coplanar connectivity

            try:
                grow = GrowTowards(
                    root_frame=f0,
                    target_frame=f1,
                    offset_root_child=offset01 * self.stick_length,
                    offset_target_child=offset01 * self.stick_length,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth,
                    face_index_root=0,      # +Y
                    face_index_target=2      # -Y
                )
                sticks_out.extend(grow.sticks)
            except Exception as e:
                print("GrowTowards failed on edge ({}, {}): {}".format(i, j, e))
                continue

        self.sticks = sticks_out
        return sticks_out

    # ------------------------------------------------------------------  
    # COLLISION DETECTION (OBB-lite via centerlines)
    # ------------------------------------------------------------------  

    def detect_collisions(self, clearance=0.0):
        """
        Approximate collisions using centerline distances and an effective
        radius ~= max(width, depth)/2 + clearance.
        """
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        radius = max(self.stick_width, self.stick_depth) * 0.5 + float(clearance)

        for i in range(n):
            li = self.sticks[i].axis
            for j in range(i + 1, n):
                lj = self.sticks[j].axis
                d = _segment_distance(li, lj, samples=7)
                if d < radius:
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------------  
    # RUN
    # ------------------------------------------------------------------  

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        clearance=0.0
    ):
        """
        Main entry point for GH.
        """
        self.surface_to_points()
        self.points_to_frames()
        self.frames_to_edgevectors()

        sticks = self.grow_sticks(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * len(self.sticks)

        return sticks
