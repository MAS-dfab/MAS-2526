# rf_core.py
# RootFrames engine for Grasshopper integration.
#
# Pipeline:
#   1. Sample points on curve / surface
#   2. Build local COMPAS Frames from Rhino frames
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


class RootFrames:

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=10,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        # geometry inputs
        self.surface_input = surface    # BrepFace or Surface (from GH script)
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

    def sample_points(self):
        pts = []
        self._curve_t = []
        self._uv_params = []
        self._rg_curve = None
        self._rg_face = None

        # --- Type normalization for surfaces ---
        surf = self.surface_input
        crv = self.curve_input

        is_curve = isinstance(crv, rg.Curve)
        is_surface = False

        face = None
        if surf is not None:
            # GH usually sends Brep; we want a BrepFace.
            if isinstance(surf, rg.Brep):
                if surf.Faces.Count > 0:
                    face = surf.Faces[0]
                    is_surface = True
            elif isinstance(surf, rg.BrepFace):
                face = surf
                is_surface = True
            elif isinstance(surf, rg.Surface):
                brep = surf.ToBrep()
                if brep.Faces.Count > 0:
                    face = brep.Faces[0]
                    is_surface = True

        # ------------------------
        # CURVE MODE
        # ------------------------
        if is_curve and not is_surface:
            crv = crv
            self._rg_curve = crv

            # Rhino 7/8 / RhinoCode differences:
            # Domain may be a property or a method. Try both.
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

            # stabilise
            pts.sort(key=lambda p: p.Z)

        else:
            raise RuntimeError("RootFrames.sample_points: input must be Curve or Surface/Brep.")

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAME GENERATION
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        frames = []

        # CURVE MODE
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

                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)

                x.unitize()
                y.unitize()
                frames.append(Frame(pt, x, y))

        # SURFACE MODE
        elif self._rg_face and self._uv_params:
            face = self._rg_face

            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)

                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)

                x.unitize()
                y.unitize()
                frames.append(Frame(pt, x, y))

        # FALLBACK
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
            if v.length < 1e-6:
                continue
            v.unitize()

            surf_normal = f0.zaxis.copy()
            y = surf_normal.cross(v)
            if y.length < 1e-6:
                y = surf_normal.cross(Vector(1, 0, 0))
            y.unitize()

            eframes.append(Frame(p0, v, y))
            evectors.append(v)

        self.edge_frames = eframes
        self.edge_vectors = evectors

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

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None,
                       collision_safe=False, collision_clearance=0.0):

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
                parent_frame=f
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
                collision_clearance=collision_clearance
            )

            for k in range(int(steps)):
                fi = int(face_seq[k % len(face_seq)]) if face_seq else 0
                ang = float(angle_seq[k % len(angle_seq)]) if angle_seq else stick_angle

                child = B.grow_once(
                    face_index=fi,
                    stick_angle=ang,
                    existing_sticks=all_sticks,
                    collision_safe=collision_safe
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
