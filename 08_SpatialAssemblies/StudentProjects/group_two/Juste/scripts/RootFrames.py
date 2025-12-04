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
    Plane,
    Rotation,
    Transformation,
    closest_point_on_line,
)

# ============================================================================
# HELPERS
# ============================================================================


def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y


def _aabb_from_stick(stick):
    """Compute axis-aligned bounding box of an oriented stick box."""
    L = stick.length
    W = stick.width
    D = stick.depth

    hl = 0.5 * L
    hw = 0.5 * W
    hd = 0.5 * D

    # local box corners around the frame origin
    locals = [
        Point(+hl, +hw, +hd),
        Point(+hl, +hw, -hd),
        Point(+hl, -hw, +hd),
        Point(+hl, -hw, -hd),
        Point(-hl, +hw, +hd),
        Point(-hl, +hw, -hd),
        Point(-hl, -hw, +hd),
        Point(-hl, -hw, -hd),
    ]

    # transform to world using the stick frame
    world_pts = [stick.frame.to_world_coordinates(p) for p in locals]

    xs = [p.x for p in world_pts]
    ys = [p.y for p in world_pts]
    zs = [p.z for p in world_pts]

    return (
        min(xs),
        max(xs),
        min(ys),
        max(ys),
        min(zs),
        max(zs),
    )


def _aabb_overlap(a, b, clearance=0.0):
    """Check if two AABBs overlap with optional clearance."""
    axmin, axmax, aymin, aymax, azmin, azmax = a
    bxmin, bxmax, bymin, bymax, bzmin, bzmax = b

    c = float(clearance)

    return (
        (axmin - c) <= (bxmax + c)
        and (axmax + c) >= (bxmin - c)
        and (aymin - c) <= (bymax + c)
        and (aymax + c) >= (bymin - c)
        and (azmin - c) <= (bzmax + c)
        and (azmax + c) >= (bzmin - c)
    )


# ============================================================================
# STICK
# ============================================================================


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


# ============================================================================
# BRANCHING MODULE  (Mode A: strict face contact, in-plane growth)
# ============================================================================


class BranchingModule:
    """
    Branch chain:

    - Each generation grows from the last stick.
    - Child near face lies exactly on a parent face (no overlap).
    - Child axis stays in the parent face plane.
    - Angle is applied *around the face normal* (no tilting back into parent).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

    # ------------------------------------------------------------------ #
    def _build_child_on_face(self, parent, face_index, stick_angle):
        fi = int(face_index) % 4
        pf = parent.frame

        # Position along the parent axis (0–1)
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = parent.axis.point_at(t)  # on parent centerline

        # Choose face normal and thickness (parent + child)
        if fi == 0:  # +Y
            n = pf.yaxis.unitized()
            half_parent = self.width * 0.5
            half_child = self.width * 0.5
        elif fi == 2:  # -Y
            n = (-pf.yaxis).unitized()
            half_parent = self.width * 0.5
            half_child = self.width * 0.5
        elif fi == 1:  # +Z
            n = pf.zaxis.unitized()
            half_parent = self.depth * 0.5
            half_child = self.depth * 0.5
        else:  # -Z
            n = (-pf.zaxis).unitized()
            half_parent = self.depth * 0.5
            half_child = self.depth * 0.5

        # Parent outer face center
        parent_face_center = axis_pt + n * half_parent
        # Child center so that its inner face touches parent outer face
        center = parent_face_center + n * half_child

        # Tangent in the face plane
        tangent = pf.xaxis
        x_in_plane = tangent - n * tangent.dot(n)
        if x_in_plane.length < 1e-6:
            x_in_plane = _stable_perp(n)
        x_in_plane.unitize()

        # Rotate inside the face plane around the normal
        angle_rad = math.radians(stick_angle)
        if abs(angle_rad) > 1e-9:
            R = Rotation.from_axis_and_angle(n, angle_rad, point=center)
            # apply rotation to x_in_plane
            temp_frame = Frame(center, x_in_plane, n)
            temp_frame.transform(R)
            x = temp_frame.xaxis.unitized()
        else:
            x = x_in_plane

        y = n
        z = x.cross(y).unitized()

        # Build axis such that center is midpoint (for consistency)
        half_len = 0.5 * self.stick_length
        start = center - x * half_len
        end = center + x * half_len
        axis = Line(start, end)

        child = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        # Override frame to guarantee correct orientation
        child.frame = Frame(center, x, y)
        return child

    # ------------------------------------------------------------------ #
    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_on_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index=face_index, stick_angle=stick_angle)


# ============================================================================
# GROW-TOWARDS (BRIDGE mode 1: face-center ↔ face-center)
# ============================================================================


class GrowTowards:
    """
    Build two child sticks starting on chosen faces of root/target frames.

    - Each child near face lies on the parent face (no overlap).
    - Face indices are opposite by default (root face vs target opposite face).
    - Children aim toward a joint point on the intersection of their face planes
      (or the midpoint between centers if intersection fails).
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

        # Child centers + normals (with proper face contact)
        c0, n0 = self._child_center_and_normal(
            self.root_frame, self.face_index_root, self.offset_root_child
        )
        c1, n1 = self._child_center_and_normal(
            self.target_frame, self.face_index_target, self.offset_target_child
        )

        # Planes for both child faces
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

    # ------------------------------------------------------------------ #
    def _child_center_and_normal(self, frame, face_index, offset_dist):
        fi = int(face_index) % 4

        # Slide along local x from the frame origin
        axis_pt = frame.point + frame.xaxis * float(offset_dist)

        if fi == 0:  # +Y
            n = frame.yaxis.unitized()
            half_parent = self.width * 0.5
            half_child = self.width * 0.5
        elif fi == 2:  # -Y
            n = (-frame.yaxis).unitized()
            half_parent = self.width * 0.5
            half_child = self.width * 0.5
        elif fi == 1:  # +Z
            n = frame.zaxis.unitized()
            half_parent = self.depth * 0.5
            half_child = self.depth * 0.5
        else:  # -Z
            n = (-frame.zaxis).unitized()
            half_parent = self.depth * 0.5
            half_child = self.depth * 0.5

        parent_face_center = axis_pt + n * half_parent
        center = parent_face_center + n * half_child
        return center, n

    # ------------------------------------------------------------------ #
    def _build_child(self, center, n, joint):
        # Direction from center toward joint, but projected into face plane
        v = Vector.from_start_end(center, joint)
        v_in_plane = v - n * v.dot(n)
        if v_in_plane.length < 1e-6:
            v_in_plane = _stable_perp(n)
        v_in_plane.unitize()

        x = v_in_plane
        y = n
        z = x.cross(y).unitized()

        half_len = 0.5 * self.len
        start = center - x * half_len
        end = center + x * half_len
        axis = Line(start, end)

        stick = Stick(axis, length=self.len, width=self.width, depth=self.depth)
        stick.frame = Frame(center, x, y)
        return stick

    # ------------------------------------------------------------------ #
    def visualize(self):
        return [s.geometry for s in self.sticks]


# ============================================================================
# ROOTFRAMES ENGINE
# ============================================================================


class RootFrames:
    """
    Pipeline:
      1) Surface/Curve → points
      2) Points → 3D frames (true curve/surface frames)
      3) Frames → edge frames + edge vectors
      4) Growth: branch (L-system) or bridge (GrowTowards)
      5) Optional: collision detection (AABB per stick)
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
        self.collision_flags = []

        # Rhino param storage
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ------------------------------------------------------------------ #
    # BLOCK 1 – SAMPLING
    # ------------------------------------------------------------------ #
    def surface_to_points(self):
        import Rhino.Geometry as rg

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

        # Surface/Brep mode
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

    # ------------------------------------------------------------------ #
    # BLOCK 2 – FRAMES
    # ------------------------------------------------------------------ #
    def points_to_frames(self, rot_tan=0.0, rot_norm=0.0):
        import Rhino.Geometry as rg

        N = len(self.points)
        if N == 0:
            self.frames = []
            return []

        frames = []

        # Curve mode: Curve.FrameAt(t)
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    tangent = crv.TangentAt(t)
                    tvec = Vector(tangent.X, tangent.Y, tangent.Z)
                    if tvec.length < 1e-6:
                        tvec = Vector(1, 0, 0)
                    else:
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

                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        # Surface mode: BrepFace.FrameAt(u, v)
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
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        else:
            # Fallback
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                frames.append(f)

        self.frames = frames
        return frames

    # ------------------------------------------------------------------ #
    # BLOCK 3 – EDGE FRAMES & VECTORS
    # ------------------------------------------------------------------ #
    def frames_to_edgevectors(self):
        pts = [f.point for f in self.frames]
        N = len(pts)

        if N < 2:
            self.edge_frames = []
            self.edge_vectors = []
            self.edges = []
            return [], []

        # basic nearest-neighbour edges
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

            # TRUE 3D direction for root sticks
            x = v.unitized()
            y = _stable_perp(x)
            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------------ #
    # BLOCK 4 – GROWTH
    # ------------------------------------------------------------------ #
    def grow_sticks(
        self,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None,
    ):
        mode = str(mode).strip().lower()
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
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

        # Branch mode
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
                sticks_out.extend(mod.sticks[1:])  # skip duplicated root
            self.sticks = sticks_out
            return sticks_out

        # Bridge mode
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

    # ------------------------------------------------------------------ #
    # COLLISION DETECTION
    # ------------------------------------------------------------------ #
    def detect_collisions(self, clearance=0.0):
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        aabbs = [_aabb_from_stick(s) for s in self.sticks]

        for i in range(n):
            for j in range(i + 1, n):
                if _aabb_overlap(aabbs[i], aabbs[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------------ #
    # RUN
    # ------------------------------------------------------------------ #
    def run(
        self,
        rot_tan=0.0,
        rot_norm=0.0,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None,
        detect=False,
        detect_collisions=None,
        clearance=0.0,
    ):
        """
        detect / detect_collisions:
            - if True, run AABB-based collision detection
        """
        if detect_collisions is None:
            detect_collisions = bool(detect)

        self.surface_to_points()
        self.points_to_frames(rot_tan, rot_norm)
        self.frames_to_edgevectors()

        sticks = self.grow_sticks(
            mode=mode,
            face_index=face_index,
            angle=angle,
            offset01=offset01,
            steps=steps,
            bridge_index=bridge_index,
        )

        if detect_collisions:
            self.detect_collisions(clearance=clearance)

        return sticks
