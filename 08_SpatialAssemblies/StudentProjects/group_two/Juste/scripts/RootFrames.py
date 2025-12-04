# RootFrames.py
# r: compas>=2.14.1
import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box,
    Plane, Rotation, Transformation, closest_point_on_line
)

# ============================================================================
# HELPERS – stable perpendicular
# ============================================================================

def _stable_perp(xaxis):
    """Return a stable perpendicular vector for a given x-axis."""
    worldZ = Vector(0, 0, 1)
    worldY = Vector(0, 1, 0)
    up = worldZ if abs(xaxis.dot(worldZ)) < 0.9 else worldY
    y = up.cross(xaxis)
    y.unitize()
    return y

# ============================================================================
# HELPERS – sampled line distance (fallback)
# ============================================================================

def _distance_point_segment(pt, line):
    p0 = line.start
    p1 = line.end
    u = p1 - p0
    uu = u.dot(u)
    if uu < 1e-12:
        return pt.distance_to_point(p0)

    t = (pt - p0).dot(u) / uu
    if t <= 0:
        cp = p0
    elif t >= 1:
        cp = p1
    else:
        cp = p0 + u * t
    return pt.distance_to_point(cp)


def _segment_distance(line1, line2):
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

# ============================================================================
# AABB HELPERS (SOLID COLLISION)
# ============================================================================

def _aabb_from_box(box):
    """Return ((minx,miny,minz),(maxx,maxy,maxz)) from oriented Box."""
    verts = [v for v in box.vertices]
    xs = [v.x for v in verts]
    ys = [v.y for v in verts]
    zs = [v.z for v in verts]
    return (
        (min(xs), min(ys), min(zs)),
        (max(xs), max(ys), max(zs)),
    )


def _aabb_overlap(a, b):
    """Return True if two AABBs overlap."""
    (ax0, ay0, az0), (ax1, ay1, az1) = a
    (bx0, by0, bz0), (bx1, by1, bz1) = b

    return (
        ax0 <= bx1 and ax1 >= bx0 and
        ay0 <= by1 and ay1 >= by0 and
        az0 <= bz1 and az1 >= bz0
    )

# ============================================================================
# STICK CLASS
# ============================================================================

class Stick:
    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH  = DEFAULT_SIZE
    DEPTH  = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None):
        """
        axis : Line (centerline)
        """
        self.axis = axis
        self.length = length or Stick.LENGTH
        self.width  = width  or Stick.WIDTH
        self.depth  = depth  or Stick.DEPTH
        self.frame  = self.compute_frame()

    def compute_frame(self):
        x = self.axis.direction.unitized()
        y = _stable_perp(x)
        z = x.cross(y).unitized()
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return oriented Box representing the stick."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box

# ============================================================================
# BRANCH MODULE – TRUE FACE CONTACT
# ============================================================================

class BranchingModule:
    """
    Child grows from parent face with true contact.
    Designer controls angle (yaw on face).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width  = width  or Stick.WIDTH
        self.depth  = depth  or Stick.DEPTH
        self.offset01 = float(offset01)

    # ------------------------------------------------------------
    # Build child
    # ------------------------------------------------------------
    def _build_child_from_face(self, parent, fi, stick_angle):
        pf = parent.frame
        fi = int(fi) % 4

        # position along parent axis
        t = max(0, min(1, self.offset01))
        axis_pt = parent.axis.point_at(t)

        # determine normal + thickness
        if fi == 0:      # +Y
            n = pf.yaxis.unitized()
            parent_half = self.width * 0.5
            child_half  = self.width * 0.5
        elif fi == 2:    # -Y
            n = (-pf.yaxis).unitized()
            parent_half = self.width * 0.5
            child_half  = self.width * 0.5
        elif fi == 1:    # +Z
            n = pf.zaxis.unitized()
            parent_half = self.depth * 0.5
            child_half  = self.depth * 0.5
        else:            # -Z
            n = (-pf.zaxis).unitized()
            parent_half = self.depth * 0.5
            child_half  = self.depth * 0.5

        # parent face center
        parent_face_center = axis_pt + n * parent_half
        # child center to place its near face on parent face
        child_center = parent_face_center + n * child_half

        # tangent projected onto face plane
        tangent = pf.xaxis
        tproj = tangent - n * tangent.dot(n)
        if tproj.length < 1e-6:
            tproj = _stable_perp(n)
        tproj.unitize()

        # blend angle
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tproj * math.sin(theta)

        # remove normal component → planar local x
        d = d_raw - n * d_raw.dot(n)
        if d.length < 1e-6:
            d = tproj
        d.unitize()

        x = d
        y = n
        z = x.cross(y).unitized()
        f_child = Frame(child_center, x, y)

        half_len = self.stick_length * 0.5
        start = child_center - x * half_len
        end   = child_center + x * half_len
        axis = Line(start, end)

        s = Stick(axis, length=self.stick_length, width=self.width, depth=self.depth)
        s.frame = f_child
        return s

    # ------------------------------------------------------------
    # Single growth step
    # ------------------------------------------------------------
    def grow_once(self, face_index=0, stick_angle=0.0):
        parent = self.sticks[-1]
        child = self._build_child_from_face(parent, face_index, stick_angle)
        self.sticks.append(child)

    # ------------------------------------------------------------
    # N-step chain
    # ------------------------------------------------------------
    def grow_chain(self, steps=1, face_index=0, stick_angle=0.0):
        for _ in range(max(0, int(steps))):
            self.grow_once(face_index, stick_angle)

# ============================================================================
# GROW-TOWARDS (bridge mode)
# ============================================================================

class GrowTowards:
    """
    Build two children whose near faces lie on selected parent faces.
    Both aim toward a joint point between face planes.
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
        self.width = width  or Stick.WIDTH
        self.depth = depth  or Stick.DEPTH

        self.root_frame   = root_frame.copy()
        self.target_frame = target_frame.copy()

        self.offset_root_child   = float(offset_root_child)
        self.offset_target_child = float(offset_target_child)

        self.face_index_root = int(face_index_root) % 4
        self.face_index_target = (
            (self.face_index_root + 2) % 4
            if face_index_target is None else int(face_index_target) % 4
        )

        self.sticks = []

        c0, n0 = self._child_center_and_normal(self.root_frame,   self.face_index_root,   self.offset_root_child)
        c1, n1 = self._child_center_and_normal(self.target_frame, self.face_index_target, self.offset_target_child)

        # try to find joint via plane intersection
        plane0 = Plane(c0, n0)
        plane1 = Plane(c1, n1)
        line = plane0.intersection_with_plane(plane1)

        if line:
            joint = Point(*closest_point_on_line(c0, line))
        else:
            joint = Point(0.5*(c0.x+c1.x), 0.5*(c0.y+c1.y), 0.5*(c0.z+c1.z))

        self.sticks.append(self._build_child(c0, n0, joint))
        self.sticks.append(self._build_child(c1, n1, joint))

    def _child_center_and_normal(self, frame, fi, offset_dist):
        fi = int(fi) % 4
        axis_pt = frame.point + frame.xaxis * offset_dist

        if fi == 0:
            n = frame.yaxis.unitized()
            half = self.width * 0.5
        elif fi == 2:
            n = (-frame.yaxis).unitized()
            half = self.width * 0.5
        elif fi == 1:
            n = frame.zaxis.unitized()
            half = self.depth * 0.5
        else:
            n = (-frame.zaxis).unitized()
            half = self.depth * 0.5

        parent_face_center = axis_pt + n * half
        child_center = parent_face_center + n * half
        return child_center, n

    def _build_child(self, center, n, joint):
        v = Vector.from_start_end(center, joint)
        vproj = v - n * v.dot(n)
        if vproj.length < 1e-6:
            vproj = _stable_perp(n)

        vproj.unitize()
        x = vproj
        y = n
        z = x.cross(y).unitized()

        half_len = self.len * 0.5
        start = center - x * half_len
        end   = center + x * half_len
        axis = Line(start, end)

        s = Stick(axis, length=self.len, width=self.width, depth=self.depth)
        s.frame = Frame(center, x, y)
        return s

# ============================================================================
# ROOTFRAMES ENGINE
# ============================================================================

class RootFrames:

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

        # param storage
        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

    # ------------------------------------------------------------
    # Sampling
    # ------------------------------------------------------------
    def surface_to_points(self):
        pts = []
        self._rg_face   = None
        self._uv_params = []
        self._rg_curve  = None
        self._curve_t   = []

        # --- Curve mode -----------------------------------------------------
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

        # --- Surface/Brep mode ----------------------------------------------
        else:
            if self.surface_input is None:
                raise Exception("No surface input provided.")

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

    # ------------------------------------------------------------
    # Build 3D frames
    # ------------------------------------------------------------
    def points_to_frames(self, rot_tan=0, rot_norm=0):
        
        frames = []

        # ---- Curve FrameAt ----------------------------------------------
        if self._rg_curve is not None and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if not ok:
                    tan = crv.TangentAt(t)
                    tvec = Vector(tan.X, tan.Y, tan.Z)
                    tvec.unitize()
                    y = _stable_perp(tvec)
                    f = Frame(pt, tvec, y)
                else:
                    xaxis = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z).unitized()
                    yaxis = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z).unitized()
                    f = Frame(pt, xaxis, yaxis)

                # optional rotations
                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)

        # ---- Surface FrameAt ---------------------------------------------
# --- Surface mode: use TRUE 3D frames -----------------------------------
        elif self._rg_face is not None and self._uv_params:
            face = self._rg_face

            for pt, (u, v) in zip(self.points, self._uv_params):
                # 1. True geometric surface normal
                normal = face.NormalAt(u, v)
                n = Vector(normal.X, normal.Y, normal.Z)
                if n.length < 1e-6:
                    n = Vector(0, 0, 1)
                else:
                    n.unitize()

                # 2. Choose X-axis from the curve of iso-u or iso-v direction
                # Compute tangent in u direction
                du = face.TangentAt(u, v)[0]    # returns (du, dv)
                tx = Vector(du.X, du.Y, du.Z)
                if tx.length < 1e-6:
                    # if tangent is degenerate, fallback
                    tx = _stable_perp(n)
                tx.unitize()

                # 3. Ensure orthogonal frame
                ty = n.cross(tx).unitized()

                f = Frame(pt, tx, ty)

                # apply user rotations
                if rot_tan:
                    R = Rotation.from_axis_and_angle(f.xaxis, math.radians(rot_tan), point=pt)
                    f.transform(R)
                if rot_norm:
                    R = Rotation.from_axis_and_angle(f.yaxis, math.radians(rot_norm), point=pt)
                    f.transform(R)

                frames.append(f)


    # ------------------------------------------------------------
    # Build edge frames
    # ------------------------------------------------------------
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
            jbest = None
            for j in range(N):
                if i == j: continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    best = d
                    jbest = j
            if jbest is not None:
                edges.add(tuple(sorted((i, jbest))))

        edges = list(edges)
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
                x = f.xaxis
            x.unitize()
            y = z.cross(x).unitized()

            eframes.append(Frame(p0, x, y))
            evectors.append(x)

        self.edge_frames = eframes
        self.edge_vectors = evectors
        return eframes, evectors

    # ------------------------------------------------------------
    # Grow sticks
    # ------------------------------------------------------------
    def grow_sticks(
        self,
        mode="branch",
        face_index=0,
        angle=0.0,
        offset01=1.0,
        steps=1,
        bridge_index=None
    ):
        mode = mode.lower()
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            return sticks_out

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            r = Stick(axis, length=self.stick_length,
                      width=self.stick_width, depth=self.stick_depth)
            roots.append(r)
            sticks_out.append(r)

        # --- BRANCH MODE ----------------------------------------------------
        if mode == "branch":
            for r in roots:
                mod = BranchingModule(
                    r,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth,
                    offset01=offset01
                )
                mod.grow_chain(steps, face_index, angle)
                sticks_out.extend(mod.sticks[1:])
            self.sticks = sticks_out
            return sticks_out

        # --- BRIDGE MODE ----------------------------------------------------
        if mode == "bridge":
            offset_abs = offset01 * self.stick_length

            for (i, j) in self.edges:
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
                    print("GrowTowards error on edge ({}, {}): {}".format(i, j, e))

            self.sticks = sticks_out
            return sticks_out

        raise Exception("Unknown mode {}".format(mode))

    # ------------------------------------------------------------
    # AABB COLLISION DETECTION
    # ------------------------------------------------------------
    def detect_collisions(self, clearance=0.0):
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collisions = flags
            return flags

        clear = float(clearance)
        aabbs = []

        # Build AABBs for each stick
        for s in self.sticks:
            box = s.geometry
            # inflate box
            box.xsize += clear
            box.ysize += clear
            box.zsize += clear
            aabbs.append(_aabb_from_box(box))

        # Pairwise check
        for i in range(n):
            for j in range(i+1, n):
                if _aabb_overlap(aabbs[i], aabbs[j]):
                    flags[i] = True
                    flags[j] = True

        self.collisions = flags
        return flags

    # ------------------------------------------------------------
    # RUN
    # ------------------------------------------------------------
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
            self.detect_collisions(clearance)

        return sticks
