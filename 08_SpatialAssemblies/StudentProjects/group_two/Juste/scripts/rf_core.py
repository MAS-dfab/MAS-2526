# ================================================================
# rf_core.py
# Clean RootFrames engine for Grasshopper
# Fully patched for curve/surface domain handling and safe typing
# ================================================================

import random
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


# ================================================================
# RootFrames CLASS
# ================================================================

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
        self.surface_input = surface
        self.curve_input = curve

        # sampling
        self.point_density = int(point_density)

        # stick geometry
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # storage
        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # sampling metadata
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []


    # ================================================================
    # 1. SAMPLE POINTS
    # ================================================================
    def sample_points(self):
        """Sample points on a curve or a surface with correct type detection."""

        pts = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # determine true types
        is_curve = isinstance(self.curve_input, rg.Curve)
        is_surface = isinstance(self.surface_input, rg.Surface)

        # ------------------------------------------------------------
        # CURVE MODE
        # ------------------------------------------------------------
        if is_curve and not is_surface:

            crv = self.curve_input
            self._rg_curve = crv

            # Curve.Domain is a PROPERTY (Interval), not a method
            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)

                pts.append(p)
                self._curve_t.append(t)

        # ------------------------------------------------------------
        # SURFACE MODE
        # ------------------------------------------------------------
        elif is_surface:

            brep = self.surface_input.ToBrep()
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

            pts.sort(key=lambda P: P.Z)

        # ------------------------------------------------------------
        # FALLBACK
        # ------------------------------------------------------------
        else:
            # no valid geometry → empty list
            self.points = []
            return []

        # convert to COMPAS points
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points


    # ================================================================
    # 2. FRAMES FROM GEOMETRY
    # ================================================================
    def frames_from_geometry(self):

        frames = []

        # curve mode
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):

                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(*plane.XAxis)
                    y = Vector(*plane.YAxis)
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

        # surface mode
        elif self._rg_face and self._uv_params:

            face = self._rg_face

            for pt, (u, v) in zip(self.points, self._uv_params):

                ok, plane = face.FrameAt(u, v)

                if ok:
                    x = Vector(*plane.XAxis)
                    y = Vector(*plane.YAxis)
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)

                x.unitize()
                y.unitize()

                frames.append(Frame(pt, x, y))

        # fallback
        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames


    # ================================================================
    # 3. BUILD EDGE FRAMES (nearest neighbor graph)
    # ================================================================
    def frames_to_edges(self):

        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
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

            # create simple perpendicular frame
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


    # ================================================================
    # Rule parser
    # ================================================================
    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        seq = []
        for t in rule_str.split(","):
            t = t.strip()
            if not t:
                continue
            try:
                seq.append(float(t))
            except:
                pass
        return seq


    # ================================================================
    # 4. BRANCHING
    # ================================================================
    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # create root sticks from each edge frame
        for f, v in zip(self.edge_frames, self.edge_vectors):

            axis = Line(f.point, f.point + v * self.stick_length)

            s = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f
            )

            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

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

            self.branch_sticks.extend(B.sticks[1:])  # skip duplicate root

        return self.branch_sticks


    # ================================================================
    # 5. BRIDGING
    # ================================================================
    def grow_bridging(self):
        if not self.branch_sticks:
            self.bridge_sticks = []
            return []

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth
        )

        self.bridge_sticks = BM.build()
        return self.bridge_sticks


    # ================================================================
    # 6. COLLISION DETECTION
    # ================================================================
    def detect_collisions(self, clearance=0.0):

        sticks = self.branch_sticks + self.bridge_sticks
        n = len(sticks)
        flags = [False] * n

        for i in range(n):
            for j in range(i + 1, n):

                if sticks[i].intersects(sticks[j], clearance=clearance):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags
        return flags


    # ================================================================
    # 7. RUN PIPELINE
    # ================================================================
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

        # sampling
        self.sample_points()
        self.frames_from_geometry()
        self.frames_to_edges()

        # branching
        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
        )

        # bridging (optional)
        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []

        # collision detection (optional)
        if detect_collisions:
            self.detect_collisions(clearance=0.0)
        else:
            self.collision_flags = [False] * (
                len(self.branch_sticks) + len(self.bridge_sticks)
            )

        return self.branch_sticks + self.bridge_sticks
