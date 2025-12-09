# rf_core.py
# RootFrames engine for Grasshopper integration.
#
# Pipeline:
#   1. Sample points on curve / surface
#   2. Build COMPAS Frames from Rhino geometry
#   3. Build nearest-neighbour edge-frames
#   4. Grow branching L-system
#   5. Grow bridging (optional)
#   6. Detect collisions (optional)
#
# Debug channels (for GH):
#   self.points
#   self.frames
#   self.root_sticks
#   self.branch_sticks
#   self.bridge_sticks
#   self.collision_flags
#   self.root_frames_debug
#   self.root_axes_debug
#   self.branch_axes_debug
#   self.bridge_axes_debug

import random
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


EPS = 1e-9


class RootFrames(object):

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=None,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        # geometry inputs (from GH wrapper)
        self.surface_input = surface    # Brep / BrepFace / Surface
        self.curve_input = curve        # Curve (optional)

        # sampling
        self.point_density = int(point_density)

        # stick geometry
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # data stores
        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # debug geometry
        self.root_frames_debug = []
        self.root_axes_debug = []
        self.branch_axes_debug = []
        self.bridge_axes_debug = []

        # internals for sampling
        self._rg_curve = None
        self._curve_t = []
        self._rg_face = None
        self._uv_params = []

    # ----------------------------------------------------------------------
    # 1. POINT SAMPLING
    # ----------------------------------------------------------------------

    def _normalize_surface_input(self):
        """Turn whatever we got (Brep / Face / Surface) into a BrepFace."""
        surf = self.surface_input
        if surf is None:
            return None

        if isinstance(surf, rg.BrepFace):
            return surf

        if isinstance(surf, rg.Brep):
            if surf.Faces.Count > 0:
                return surf.Faces[0]
            return None

        if isinstance(surf, rg.Surface):
            brep = surf.ToBrep()
            if brep and brep.Faces.Count > 0:
                return brep.Faces[0]
            return None

        # unsupported
        return None

    def sample_points(self):
        pts = []
        self._curve_t = []
        self._uv_params = []
        self._rg_curve = None
        self._rg_face = None

        # NORMALIZE input types
        face = self._normalize_surface_input()
        crv = self.curve_input

        is_curve = isinstance(crv, rg.Curve)
        is_surface = face is not None

        # ------------------------
        # CURVE MODE
        # ------------------------
        if is_curve and not is_surface:
            self._rg_curve = crv

            # Rhino Domain can be property or method depending on version
            try:
                dom = crv.Domain
                t0, t1 = dom.Min, dom.Max
            except Exception:
                dom = crv.Domain()
                t0, t1 = dom.Min, dom.Max

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # ------------------------
        # SURFACE MODE
        # ------------------------
        elif is_surface:
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.Min, udom.Max)
                v = random.uniform(vdom.Min, vdom.Max)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            # optional stabilisation: sort by Z
            pts.sort(key=lambda p: p.Z)

        else:
            raise RuntimeError(
                "RootFrames.sample_points: input must be Curve or Surface/Brep."
            )

        # convert to COMPAS Points
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAME GENERATION (true surface normal, orthonormal basis)
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        """
        Build local COMPAS Frames at each sampled point.
        Handles:
            - Curve mode
            - Surface mode
            - Fallback global frame
        """
        frames = []

        # -------------------------------------------------
        # CURVE MODE
        # -------------------------------------------------
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    y = Vector(0, 0, 1).cross(x)

                if x.length < EPS:
                    x = Vector(1, 0, 0)
                x.unitize()

                if y.length < EPS:
                    y = Vector(0, 1, 0)
                y.unitize()

                frames.append(Frame(pt, x, y))

        # -------------------------------------------------
        # SURFACE MODE
        # -------------------------------------------------
        elif self._rg_face and self._uv_params:
            face = self._rg_face
            srf  = face.UnderlyingSurface()

            for pt, (u, v) in zip(self.points, self._uv_params):

                # === 1. TRUE NORMAL ===
                n = face.NormalAt(u, v)
                z = Vector(n.X, n.Y, n.Z)
                if z.length < EPS:
                    z = Vector(0, 0, 1)
                z.unitize()

                # === 2. TANGENTS USING UNIVERSAL DERIVATIVE ACCESS ===
                du = dv = None
                try:
                    # returns: (pt, d1, d2, d3, ...)
                    derivs = srf.Evaluate(u, v, 1)
                    # derivs[1] may be a Vector3d OR a list of Vector3d
                    raw_du = derivs[1]
                    raw_dv = derivs[2]

                    # unwrap arrays if necessary
                    if hasattr(raw_du, "__len__"):
                        raw_du = raw_du[0]
                    if hasattr(raw_dv, "__len__"):
                        raw_dv = raw_dv[0]

                    du = Vector(raw_du.X, raw_du.Y, raw_du.Z)
                    dv = Vector(raw_dv.X, raw_dv.Y, raw_dv.Z)

                except:
                    # fallback: fabricate orthogonal tangents
                    du = Vector(1, 0, 0)
                    dv = Vector(0, 1, 0)

                # Pick strongest tangent for x-axis
                x = du if du.length >= dv.length else dv

                # Orthonormalize x vs z
                x = x - z * x.dot(z)
                if x.length < EPS:
                    x = Vector(1, 0, 0).cross(z)
                x.unitize()

                # y = z × x
                y = z.cross(x)
                if y.length < EPS:
                    y = Vector(0, 1, 0).cross(z)
                y.unitize()

                frames.append(Frame(pt, x, y))


        # -------------------------------------------------
        # FALLBACK MODE
        # -------------------------------------------------
        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. NEAREST NEIGHBOUR EDGE-FRAMES (3D-preserving)
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            self.root_frames_debug = []
            self.root_axes_debug = []
            return [], []

        edges = set()

        # simple NN search
        for i in range(n):
            pi = pts[i]
            best = 1e9
            j_best = None

            for j in range(n):
                if i == j:
                    continue
                d = pi.distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j

            edges.add(tuple(sorted((i, j_best))))

        edges = list(edges)
        self.edges = edges

        eframes = []
        evectors = []

        for i, j in edges:
            f0 = self.frames[i]
            p0 = f0.point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < EPS:
                continue
            v.unitize()

            # use local surface normal as z-axis
            z = f0.zaxis
            if z.length < EPS:
                z = Vector(0, 0, 1)

            y = z.cross(v)
            if y.length < EPS:
                # fallback if v almost parallel to z
                y = z.cross(Vector(1, 0, 0))
                if y.length < EPS:
                    y = z.cross(Vector(0, 1, 0))
            y.unitize()

            eframes.append(Frame(p0, v, y))
            evectors.append(v)

        self.edge_frames = eframes
        self.edge_vectors = evectors

        # debug: root frames + axes
        self.root_frames_debug = list(eframes)
        self.root_axes_debug = [
            Line(f.point, f.point + v * self.stick_length)
            for f, v in zip(eframes, evectors)
        ]

        return eframes, evectors

    # ----------------------------------------------------------------------
    # Utility
    # ----------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        vals = []
        for tok in rule_str.split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except Exception:
                pass
        return vals

    # ----------------------------------------------------------------------
    # 4. BRANCHING (with optional collision-safe growth)
    # ----------------------------------------------------------------------

    def grow_branching(
        self,
        steps,
        stick_angle,
        offset01,
        face_rule=None,
        angle_rule=None,
        collision_safe=False,
        collision_clearance=0.0,
    ):

        self.root_sticks = []
        self.branch_sticks = []
        self.branch_axes_debug = []

        all_sticks = []

        # root sticks from edge frames
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            s = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f,
            )
            s.is_root = True
            self.root_sticks.append(s)
            self.branch_sticks.append(s)
            all_sticks.append(s)

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
                collision_clearance=collision_clearance,
            )

            for k in range(int(steps)):
                fi = int(face_seq[k % len(face_seq)]) if face_seq else 0
                ang = float(angle_seq[k % len(angle_seq)]) if angle_seq else stick_angle

                child = B.grow_once(
                    face_index=fi,
                    stick_angle=ang,
                    existing_sticks=all_sticks,
                    collision_safe=collision_safe,
                )

                if child is not None:
                    self.branch_sticks.append(child)
                    all_sticks.append(child)
                    self.branch_axes_debug.append(child.axis)

        return self.branch_sticks

    # ----------------------------------------------------------------------
    # 5. BRIDGING
    # ----------------------------------------------------------------------

    def grow_bridging(self):
        if not self.branch_sticks:
            self.bridge_sticks = []
            self.bridge_axes_debug = []
            return []

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
        )

        bridges = BM.build()
        for br in bridges:
            br.is_bridge = True
            br.family = "BRIDGE"

        self.bridge_sticks = bridges
        self.bridge_axes_debug = [b.axis for b in bridges]
        return bridges

    # ----------------------------------------------------------------------
    # 6. COLLISION DETECTION
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)
        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):
                if sticks[i].intersects(sticks[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        for flag, s in zip(flags, sticks):
            s.collided = flag

        self.collision_flags = flags
        return flags

    # ----------------------------------------------------------------------
    # 7. RUN PIPELINE
    # ----------------------------------------------------------------------

    def run(
        self,
        steps=1,
        stick_angle=0.0,
        offset01=0.5,
        detect_collisions=False,
        do_bridging=False,
        face_rule=None,
        angle_rule=None,
        debug=False,
        collision_clearance=0.0,
    ):

        self.sample_points()
        self.frames_from_geometry()
        self.frames_to_edges()

        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
            collision_safe=detect_collisions,
            collision_clearance=collision_clearance,
        )

        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []
            self.bridge_axes_debug = []

        if detect_collisions:
            self.detect_collisions(clearance=collision_clearance)
        else:
            sticks = self.branch_sticks + self.bridge_sticks
            self.collision_flags = [False] * len(sticks)
            for s in sticks:
                s.collided = False

        return self.branch_sticks + self.bridge_sticks
