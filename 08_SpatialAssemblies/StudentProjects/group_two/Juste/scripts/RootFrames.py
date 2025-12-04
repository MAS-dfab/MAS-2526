# RootFrames.py
# r: compas>=2.14.1

import math
import random

from compas.geometry import (
    Point, Vector, Frame, Line, Box,
    Rotation, Transformation,
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
    """Oriented box represented by a centerline axis and a local frame."""

    DEFAULT_LEN = 100.0
    DEFAULT_SIZE = 5.0

    LENGTH = DEFAULT_LEN
    WIDTH = DEFAULT_SIZE
    DEPTH = DEFAULT_SIZE

    def __init__(self, axis, length=None, width=None, depth=None):
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
        self.frame = self.compute_frame()

    def compute_frame(self):
        """Compute a local frame from the axis (never from the surface)."""
        x = self.axis.direction.unitized()
        y = _stable_perp(x)
        z = x.cross(y).unitized()
        return Frame(self.axis.midpoint, x, y)

    @property
    def geometry(self):
        """Return an oriented Box for this stick."""
        box = Box(self.axis.length, self.width, self.depth)
        T = Transformation.from_frame_to_frame(Frame.worldXY(), self.frame)
        box.transform(T)
        return box

    # --- AABB for collision detection ---------------------------------

    def aabb(self, clearance=0.0):
        """
        Compute an axis-aligned bounding box (AABB) of this oriented stick.
        Returns (minx, maxx, miny, maxy, minz, maxz).
        """
        xaxis = self.frame.xaxis
        yaxis = self.frame.yaxis
        zaxis = self.frame.zaxis

        hx = 0.5 * self.length
        hy = 0.5 * self.width
        hz = 0.5 * self.depth

        corners = []
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                for sz in (-hz, hz):
                    p = (
                        self.frame.point
                        + xaxis * sx
                        + yaxis * sy
                        + zaxis * sz
                    )
                    corners.append(p)

        minx = min(p.x for p in corners) - clearance
        maxx = max(p.x for p in corners) + clearance
        miny = min(p.y for p in corners) - clearance
        maxy = max(p.y for p in corners) + clearance
        minz = min(p.z for p in corners) - clearance
        maxz = max(p.z for p in corners) + clearance

        return (minx, maxx, miny, maxy, minz, maxz)


# =============================================================================
# BRANCHING MODULE
# =============================================================================

class BranchingModule:
    """
    Branch chain:
      - Each generation grows from the last stick.
      - Child near face lies exactly on a parent face (no overlap).
      - Child axis is a blend of parent tangent and face normal (true 3D).
    """

    def __init__(self, root_stick, stick_length=None, width=None, depth=None, offset01=0.5):
        self.sticks = [root_stick]
        self.stick_length = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH
        self.offset01 = float(offset01)

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
            half_parent = parent.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 2:        # -Y
            n = (-pf.yaxis).unitized()
            half_parent = parent.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 1:        # +Z
            n = pf.zaxis.unitized()
            half_parent = parent.depth * 0.5
            half_child  = self.depth * 0.5
        else:                # -Z
            n = (-pf.zaxis).unitized()
            half_parent = parent.depth * 0.5
            half_child  = self.depth * 0.5

        # strict face contact:
        parent_face_center = axis_pt + n * half_parent
        child_center = parent_face_center + n * half_child

        # blend normal + tangent (true 3D, no projection back to a plane)
        tangent = pf.xaxis
        theta = math.radians(stick_angle)
        d_raw = n * math.cos(theta) + tangent * math.sin(theta)
        if d_raw.length < 1e-6:
            d_raw = n
        d_raw.unitize()

        # ensure we are growing away from the parent
        if d_raw.dot(n) <= 0:
            d_raw = -d_raw

        x = d_raw
        # try to keep y ~ normal, but keep frame orthogonal
        y = n
        z = x.cross(y)
        if z.length < 1e-6:
            y = _stable_perp(x)
            z = x.cross(y)
        z.unitize()
        y = z.cross(x)
        y.unitize()

        child_frame = Frame(child_center, x, y)

        half_len = 0.5 * self.stick_length
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
# BRIDGING (GrowBridge) – after branching
# =============================================================================

class GrowBridge:
    """
    Build two child sticks starting on faces of two sticks (A,B).
    Near faces lie on parent faces; axes aim toward the midpoint
    between the two child centers.
    """

    def __init__(
        self,
        stick_a,
        stick_b,
        offset01=0.5,
        stick_length=None,
        width=None,
        depth=None,
        face_index_a=0,
        face_index_b=2
    ):
        self.len   = stick_length or Stick.LENGTH
        self.width = width or Stick.WIDTH
        self.depth = depth or Stick.DEPTH

        self.stick_a = stick_a
        self.stick_b = stick_b
        self.offset01 = float(offset01)

        self.face_index_a = int(face_index_a) % 4
        self.face_index_b = int(face_index_b) % 4

        self.sticks = []

        ca, na = self._child_center_and_normal(stick_a, self.face_index_a)
        cb, nb = self._child_center_and_normal(stick_b, self.face_index_b)

        joint = Point(
            0.5 * (ca.x + cb.x),
            0.5 * (ca.y + cb.y),
            0.5 * (ca.z + cb.z),
        )

        self.sticks.append(self._build_child(ca, na, joint))
        self.sticks.append(self._build_child(cb, nb, joint))

    def _child_center_and_normal(self, stick, face_index):
        fi = int(face_index) % 4
        f = stick.frame

        # position along axis
        t = max(0.0, min(1.0, self.offset01))
        axis_pt = stick.axis.point_at(t)

        if fi == 0:          # +Y
            n = f.yaxis.unitized()
            half_parent = stick.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 2:        # -Y
            n = (-f.yaxis).unitized()
            half_parent = stick.width * 0.5
            half_child  = self.width * 0.5
        elif fi == 1:        # +Z
            n = f.zaxis.unitized()
            half_parent = stick.depth * 0.5
            half_child  = self.depth * 0.5
        else:                # -Z
            n = (-f.zaxis).unitized()
            half_parent = stick.depth * 0.5
            half_child  = self.depth * 0.5

        parent_face_center = axis_pt + n * half_parent
        child_center = parent_face_center + n * half_child
        return child_center, n

    def _build_child(self, center, n, joint):
        v = Vector.from_start_end(center, joint)
        if v.length < 1e-6:
            v = _stable_perp(n)
        v.unitize()

        x = v
        y = n
        z = x.cross(y)
        if z.length < 1e-6:
            y = _stable_perp(x)
            z = x.cross(y)
        z.unitize()
        y = z.cross(x)
        y.unitize()

        half_len = 0.5 * self.len
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
      2) Points → 3D frames (root frames)
      3) Frames → neighbour edges + edge frames
      4) Growth:
           - Branching (face-contact chains) from each root stick
           - Bridging between spatially nearest, non-parallel root sticks
      5) Optional: collision detection via AABB on oriented sticks
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
        self.root_sticks  = []
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

            # normalize to Brep
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
    # BLOCK 2 – 3D FRAMES (ROOT)
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
            udom = face.Domain(0)
            vdom = face.Domain(1)
            for pt, (u, v) in zip(self.points, self._uv_params):
                # surface normal
                nvec = face.NormalAt(u, v)
                n = Vector(nvec.X, nvec.Y, nvec.Z)
                if n.length < 1e-6:
                    n = Vector(0, 0, 1)
                else:
                    n.unitize()

                # approximate tangent along U
                du = (udom.T1 - udom.T0) * 1e-3
                pu = face.PointAt(u + du, v)
                tu = Vector(pu.X - pt.x, pu.Y - pt.y, pu.Z - pt.z)
                if tu.length < 1e-6:
                    tu = _stable_perp(n)
                else:
                    tu.unitize()

                x = tu
                y = n.cross(x)
                if y.length < 1e-6:
                    y = _stable_perp(x)
                else:
                    y.unitize()
                z = x.cross(y)
                z.unitize()
                # re-orthogonalize y
                y = z.cross(x)
                y.unitize()

                f = Frame(pt, x, y)
                frames.append(f)

        else:
            # fallback
            for pt in self.points:
                f = Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0))
                frames.append(f)

        self.frames = frames
        return frames

    # ------------------------------------------------------------------  
    # BLOCK 3 – EDGE FRAMES & VECTORS
    # ------------------------------------------------------------------  

    def frames_to_edgevectors(self):
        """Nearest-neighbour edges + edge frames (3D)."""
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
    # BLOCK 4 – GROWTH (BRANCH + BRIDGE)
    # ------------------------------------------------------------------  

    def grow_sticks(self, steps=1, stick_angle=0.0, offset01=0.5):
        """
        Create:
          - root sticks along edge frames
          - branching chains from each root
          - bridging between spatially nearest, non-parallel root sticks
        """
        sticks_out = []

        if not self.edge_frames:
            self.sticks = []
            self.root_sticks = []
            return sticks_out

        # ---------- ROOT STICKS + BRANCHING ----------------------------

        roots = []
        for f, v in zip(self.edge_frames, self.edge_vectors):
            half_len = 0.5 * self.stick_length
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

        self.root_sticks = roots

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

        # ---------- BRIDGING BETWEEN ROOT STICKS -----------------------

        angle_threshold_deg = 10.0
        max_bridge_dist = 3.0 * self.stick_length

        n_roots = len(roots)
        for i in range(n_roots):
            si = roots[i]
            pi = si.frame.point
            ni = si.frame.zaxis.unitized()

            # find nearest j > i
            best_d = 1e9
            j_best = None
            for j in range(i + 1, n_roots):
                sj = roots[j]
                pj = sj.frame.point
                d = pi.distance_to_point(pj)
                if d < best_d:
                    best_d = d
                    j_best = j

            if j_best is None:
                continue

            if best_d > max_bridge_dist:
                continue

            sj = roots[j_best]
            pj = sj.frame.point
            nj = sj.frame.zaxis.unitized()

            cosang = max(-1.0, min(1.0, ni.dot(nj)))
            ang = math.degrees(math.acos(cosang))

            # only bridge if sufficiently non-parallel (≈ non-coplanar intent)
            if ang < angle_threshold_deg:
                continue

            try:
                bridge = GrowBridge(
                    si,
                    sj,
                    offset01=offset01,
                    stick_length=self.stick_length,
                    width=self.stick_width,
                    depth=self.stick_depth,
                    face_index_a=0,   # +Y on A
                    face_index_b=2    # -Y on B
                )
                sticks_out.extend(bridge.sticks)
            except Exception as e:
                print("GrowBridge failed for roots ({}, {}): {}".format(i, j_best, e))
                continue

        self.sticks = sticks_out
        return sticks_out

    # ------------------------------------------------------------------  
    # COLLISION DETECTION (AABB)
    # ------------------------------------------------------------------  

    def detect_collisions(self, clearance=0.0):
        """
        Approximate collisions using AABB overlap on oriented sticks.
        """
        n = len(self.sticks)
        flags = [False] * n
        if n < 2:
            self.collision_flags = flags
            return flags

        aabbs = [s.aabb(clearance=clearance) for s in self.sticks]

        for i in range(n):
            minx_i, maxx_i, miny_i, maxy_i, minz_i, maxz_i = aabbs[i]
            for j in range(i + 1, n):
                minx_j, maxx_j, miny_j, maxy_j, minz_j, maxz_j = aabbs[j]

                overlap_x = not (maxx_i < minx_j or maxx_j < minx_i)
                overlap_y = not (maxy_i < miny_j or maxy_j < miny_i)
                overlap_z = not (maxz_i < minz_j or maxz_j < minz_i)

                if overlap_x and overlap_y and overlap_z:
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags

    # ------------------------------------------------------------------  
    # RUN – entry point for GH
    # ------------------------------------------------------------------  

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        clearance=0.0
    ):
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
