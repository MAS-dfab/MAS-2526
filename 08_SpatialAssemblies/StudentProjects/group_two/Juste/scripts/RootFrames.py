# RootFrames.py
# r: compas>=2.14.1

import math
import random

import Rhino.Geometry as rg

from compas.geometry import (
    Point,
    Vector,
    Frame,
    Line,
    Box,
    Plane,
    Rotation,
    Transformation,
    closest_point_on_line,
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


def _stick_aabb(stick):
    """
    Axis-aligned bounding box for a stick, in world coordinates.

    Returns
    -------
    (minx, miny, minz), (maxx, maxy, maxz)
    """
    f = stick.frame
    x = f.xaxis
    y = f.yaxis
    z = f.zaxis

    hx = stick.length * 0.5
    hy = stick.width * 0.5
    hz = stick.depth * 0.5

    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p = f.point + x * (sx * hx) + y * (sy * hy) + z * (sz * hz)
                pts.append(p)

    minx = min(p.x for p in pts)
    miny = min(p.y for p in pts)
    minz = min(p.z for p in pts)
    maxx = max(p.x for p in pts)
    maxy = max(p.y for p in pts)
    maxz = max(p.z for p in pts)
    return (minx, miny, minz), (maxx, maxy, maxz)


def _aabb_overlap(a, b, eps=1e-6):
    """
    Return True if two AABBs overlap (with small tolerance).
    a, b : ((minx,miny,minz),(maxx,maxy,maxz))
    """
    (aminx, aminy, aminz), (amaxx, amaxy, amaxz) = a
    (bminx, bminy, bminz), (bmaxx, bmaxy, bmaxz) = b

    if amaxx < bminx - eps or bmaxx < aminx - eps:
        return False
    if amaxy < bminy - eps or bmaxy < aminy - eps:
        return False
    if amaxz < bminz - eps or bmaxz < aminz - eps:
        return False
    return True


# =============================================================================
# STICK
# =============================================================================
class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH  = DEFAULT_SIZE
    DEPTH  = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None):
        """
        Parameters
        ----------
        axis : compas.geometry.Line
            Centerline of the stick.
        length : float, optional
        width  : float, optional
        depth  : float, optional
        """
        self.axis   = axis
        self.length = length or Stick.LENGTH
        self.width  = width  or Stick.WIDTH
        self.depth  = depth  or Stick.DEPTH

        # frame is always derived from the *axis*, never from a surface
        self.frame = self.compute_frame()

    def compute_frame(self):
        """Build a local frame from the 3D axis direction."""
        x = self.axis.direction
        if not x.length:
            x = Vector(1, 0, 0)
        else:
            x.unitize()

        # choose a reference that is not parallel to x
        ref = Vector(0, 0, 1)
        if abs(ref.dot(x)) > 0.9:
            ref = Vector(0, 1, 0)

        y = ref.cross(x)
        if not y.length:
            y = _stable_perp(x)
        y.unitize()

        # z is implied, but we let Frame construct it from x,y
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return a compas Box aligned with the stick frame."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box


# =============================================================================
# BRANCHING MODULE  (L-system style, collision-aware face contact)
# =============================================================================

class BranchingModule:
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies on a parent face (full width/depth offset).
      - Child axis is tangent to the parent and rotated by stick_angle
        in the plane orthogonal to the chosen face normal.
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks       = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width        = width  or Stick.WIDTH
        self.depth        = depth  or Stick.DEPTH
        self.offset01     = float(offset01)

    # ------------------------------------------------------------------  
    def _build_child_from_face(self, parent, face_index, stick_angle):
        """Construct one child stick from a given parent face."""
        fi = int(face_index) % 4
        pf = parent.frame

        # 1) position along parent axis (0–1)
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # 2) face normal & thickness (parent and child share dimensions)
        if fi == 0:          # +Y
            n    = pf.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:        # -Y
            n    = (-pf.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:        # +Z
            n    = pf.zaxis.unitized()
            half = self.depth * 0.5
        else:                # -Z
            n    = (-pf.zaxis).unitized()
            half = self.depth * 0.5

        # parent face center (outer skin of parent box)
        parent_face_center = axis_pt + n * half
        # child center so its near face sits exactly on the parent face
        child_center = parent_face_center + n * half

        # 3) tangent direction projected to the plane orthogonal to n
        tangent = pf.xaxis
        tangent_proj = tangent - n * tangent.dot(n)
        if tangent_proj.length < 1e-6:
            tangent_proj = _stable_perp(n)
        tangent_proj.unitize()

        # 4) rotate the projected tangent around the face normal by stick_angle
        theta = math.radians(stick_angle)
        # pure rotation about n
        # d_raw = R_n(theta) * tangent_proj
        # implement as Rodrigues' formula:
        d_raw = (
            tangent_proj * math.cos(theta) +
            n.cross(tangent_proj) * math.sin(theta) +
            n * (n.dot(tangent_proj)) * (1.0 - math.cos(theta))
        )

        # ensure d is orthogonal to n (good for box cross-section)
        d = d_raw - n * d_raw.dot(n)
        if d.length < 1e-6:
            d = tangent_proj
        d.unitize()

        x = d
        y = n
        # z is implied; we keep frame orthonormal
        child_frame = Frame(child_center, x, y)

        # 5) build child axis from its center and local x-direction
        half_len = self.stick_length * 0.5
        start = child_center - x * half_len
        end   = child_center + x * half_len
        axis  = Line(start, end)

        child = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        # override its auto-computed frame with our precise one
        child.frame = child_frame
        return child

    # ------------------------------------------------------------------  
    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child  = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# =============================================================================
# GROW-TOWARDS (BRIDGING between two frames)
# =============================================================================


class GrowTowards:
    """
    Build two child sticks starting on chosen faces of two parent frames.
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
        face_index_target=None,
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

        c0, n0 = self._child_center_and_normal(
            self.root_frame, self.face_index_root, self.offset_root_child
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

        if fi == 0:  # +Y
            n = frame.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:  # -Y
            n = (-frame.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:  # +Z
            n = frame.zaxis.unitized()
            half = self.depth * 0.5
        else:  # -Z
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
        end = center + x * half_len
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
      2) Points → 3D frames (surface normals or curve frames)
      3) Frames → edge frames + edge vectors
      4) Growth: branch (L-system) or branch + bridge
      5) Optional: collision detection for resulting sticks (AABB)
    """

    def __init__(
        self,
        surface=None,
        curve=None,
        height_subdiv=5,
        point_density=10,
        twist_angle=0.0,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
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
        self.collisions = []

        # for surface / curve sampling
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ------------------------------------------------------------------
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------

    def surface_to_points(self):
        pts = []

        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # Curve mode
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv
            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            count = max(1, self.point_density * max(1, self.height_subdiv))
            for _ in range(count):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # Surface / Brep mode
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

            # sort by height to give some vertical ordering
            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------
    # BLOCK 2 – 3D FRAMES
    # ------------------------------------------------------------------

    def points_to_frames(self, rot_tan=0.0, rot_norm=0.0):
        N = len(self.points)
        if N == 0:
            self.frames = []
            return []

        frames = []

        # Curve mode
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve
            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    tangent = crv.TangentAt(t)
                    tv = Vector(tangent.X, tangent.Y, tangent.Z)
                    if tv.length < 1e-6:
                        tv = Vector(1, 0, 0)
                    else:
                        tv.unitize()
                    y = _stable_perp(tv)
                    f = Frame(pt, tv, y)
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

                # optional twist about X
                if self.twist_angle:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(self.twist_angle), point=pt)
                    f.transform(R)

                # optional user rotations
                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        # Surface/Brep mode: use surface normals (3D correct!)
        elif self._rg_face is not None and self._uv_params:
            face = self._rg_face
            for pt, (u, v) in zip(self.points, self._uv_params):
                nvec = face.NormalAt(u, v)
                n = Vector(nvec.X, nvec.Y, nvec.Z)
                if n.length < 1e-6:
                    n = Vector(0, 0, 1)
                else:
                    n.unitize()

                x = n
                y = _stable_perp(x)
                f = Frame(pt, x, y)

                # optional twist about X
                if self.twist_angle:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(self.twist_angle), point=pt)
                    f.transform(R)

                # optional user rotations
                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        else:
            # Fallback: world frames
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
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
            if j_best is not None:
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
    ):
        """
        Parameters
        ----------
        mode : str
            'branch' → branching only
            'bridge' → branching + bridging
        face_index : int
            0..3 which face to branch/bridge from.
        angle : float
            Branching angle in degrees.
        offset01 : float
            [0,1] param along axis for branching, mapped to abs offset for bridging.
        steps : int
            Number of branching steps per root.
        """
        mode = str(mode).strip().lower()
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            return sticks_out

        # root sticks
        root_sticks = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
            )
            root_sticks.append(root)
            sticks_out.append(root)

        # ---- BRANCH PHASE ------------------------------------------------
        for r in root_sticks:
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
            sticks_out.extend(mod.sticks[1:])  # drop duplicated root

        # ---- BRIDGE PHASE (optional) ------------------------------------
        if mode == "bridge":
            offset_abs = offset01 * self.stick_length

            # use the *final* frames of each root chain for bridging
            chain_ends = root_sticks  # simple: still use root frames for now
            num = len(chain_ends)

            for i in range(num):
                for j in range(i + 1, num):
                    fi = chain_ends[i].frame
                    fj = chain_ends[j].frame

                    # ignore nearly coplanar frames (bridge only non-coplanar-ish)
                    dotz = abs(fi.zaxis.dot(fj.zaxis))
                    if dotz > 0.99:
                        continue

                    try:
                        grow = GrowTowards(
                            root_frame=fi,
                            target_frame=fj,
                            offset_root_child=offset_abs,
                            offset_target_child=offset_abs,
                            stick_length=self.stick_length,
                            width=self.stick_width,
                            depth=self.stick_depth,
                            face_index_root=face_index,
                        )
                        sticks_out.extend(grow.sticks)
                    except Exception as e:
                        print("GrowTowards failed on pair ({}, {}): {}".format(i, j, e))
                        continue

        self.sticks = sticks_out
        return sticks_out

    # ------------------------------------------------------------------
    # COLLISION DETECTION (AABB)
    # ------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        """
        Approximate collisions using axis-aligned bounding boxes.

        Parameters
        ----------
        clearance : float
            Extra distance added to the stick dimensions before flagging a collision.
        """
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collisions = flags
            return flags

        aabbs = []
        for s in self.sticks:
            (minx, miny, minz), (maxx, maxy, maxz) = _stick_aabb(s)
            # expand by clearance
            minx -= clearance
            miny -= clearance
            minz -= clearance
            maxx += clearance
            maxy += clearance
            maxz += clearance
            aabbs.append(((minx, miny, minz), (maxx, maxy, maxz)))

        for i in range(n):
            for j in range(i + 1, n):
                if _aabb_overlap(aabbs[i], aabbs[j]):
                    flags[i] = True
                    flags[j] = True

        self.collisions = flags
        return flags

    # ------------------------------------------------------------------
    # RUN
    # ------------------------------------------------------------------

    def run(
        self,
        rot_tan=0.0,
        rot_norm=0.0,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        detect_collisions=False,
        clearance=0.0,
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
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)

        return sticks
