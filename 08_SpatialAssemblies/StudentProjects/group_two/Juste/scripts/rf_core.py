# rf_core.py
# Clean, synchronized RootFrames engine for Grasshopper integration
# Compatible with updated stick_fixed, branch, bridge

import random
import Rhino.Geometry as rg  # type: ignore

from compas.geometry import Point, Vector, Line, Frame

# crucial — you were missing this
from stick_fixed import Stick

from branch import BranchingModule
from bridge import BridgingModule


class RootFrames:
    """
    RootFrames engine:

        1) Random point sampling on curve or surface
        2) Local frame extraction (Rhino)
        3) Nearest-neighbor edges (for root sticks)
        4) Branching (L-system, face rules, angle rules, offsets)
        5) Optional bridging
        6) Optional collision detection
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
        self.curve_input = curve

        self.point_density = int(point_density)

        # stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # storage
        self.points = []
        self.frames = []
        self.edges = []
        self.edge_frames = []
        self.edge_vectors = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # helpers for sampling
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ----------------------------------------------------------------------
    # 1. POINT SAMPLING
    # ----------------------------------------------------------------------

    def sample_points(self):
        pts = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # CURVE MODE -----------------------------------------
        if self.curve_input and not self.surface_input:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # SURFACE MODE ---------------------------------------
        else:
            surf = self.surface_input
            brep = surf.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(max(1, self.point_density)):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

            # optional meaningful ordering
            pts.sort(key=lambda p: p.Z)

        # convert to COMPAS Points
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAME GENERATION
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        frames = []

        # CURVE MODE -----------------------------------------
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:  # fallback
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

        # SURFACE MODE ---------------------------------------
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

                x.unitize()
                y.unitize()

                frames.append(Frame(pt, x, y))

        # FALLBACK -------------------------------------------
        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. NEAREST NEIGHBOR EDGES
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            return [], []

        edges = set()

        # brute nearest-neighbor
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

            # perpendicular construction
            z = Vector(0, 0, 1)
            y = z.cross(v)
            if y.length < 1e-6:
                y = Vector(0, 1, 0)
            y.unitize()

            eframes.append(Frame(p0, v, y))
            evectors.append(v)

        self.edge_frames = eframes
        self.edge_vectors = evectors

        return eframes, evectors

    # ----------------------------------------------------------------------
    # Utility for turning comma-separated rules into floats
    # ----------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        vals = []
        for tok in rule_str.split(","):
            tok = tok.strip()
            if tok:
                try:
                    vals.append(float(tok))
                except:
                    pass
        return vals

    # ----------------------------------------------------------------------
    # 4. BRANCHING
    # ----------------------------------------------------------------------

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # build root sticks (one per edge)
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)

            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f
            )

            self.root_sticks.append(root)
            self.branch_sticks.append(root)

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # per-stick branching
        for root in self.root_sticks:

            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01
            )

            for k in range(int(steps)):
                fi = int(face_seq[k % len(face_seq)]) if face_seq else 0
                ang = float(angle_seq[k % len(angle_seq)]) if angle_seq else stick_angle

                B.grow_once(face_index=fi, stick_angle=ang)

            # add all children except root
            if len(B.sticks) > 1:
                self.branch_sticks.extend(B.sticks[1:])

        return self.branch_sticks

    # ----------------------------------------------------------------------
    # 5. BRIDGING
    # ----------------------------------------------------------------------

    def grow_bridging(self):
        if not self.branch_sticks:
            self.bridge_sticks = []
            return []

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
        )

        self.bridge_sticks = BM.build()
        return self.bridge_sticks

    # ----------------------------------------------------------------------
    # 6. COLLISION DETECTION
    # ----------------------------------------------------------------------

    def detect_collisions(self, clearance=0.0):
        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)

        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):
                if sticks[i].intersects(sticks[j], clearance):
                    flags[i] = True
                    flags[j] = True

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
        )

        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []

        if detect_collisions:
            self.detect_collisions()
        else:
            self.collision_flags = [False] * (
                len(self.branch_sticks) + len(self.bridge_sticks)
            )

        return self.branch_sticks + self.bridge_sticks
