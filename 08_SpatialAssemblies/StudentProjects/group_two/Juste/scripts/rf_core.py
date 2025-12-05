# rf_core.py
# Clean, stable RootFrames core
# Uses 3D-aware Stick and BranchingModule

import random
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


class RootFrames:
    """
    RootFrames engine:

      1) Sample points on a curve or surface
      2) Build 3D frames using Rhino's native frames
      3) Build nearest-neighbour edges & edge-frames
      4) Branching phase (L-system style rules)
      5) Optional bridging (non-coplanar sticks)
      6) Optional collision detection via Stick AABBs

    Debug/data channels:
      - self.root_sticks
      - self.branch_sticks
      - self.bridge_sticks
      - self.collision_flags
      - self.frames
      - self.edge_frames
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
        # geometry inputs
        self.surface_input = surface
        self.curve_input = curve

        # sampling density
        self.point_density = int(point_density)

        # stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # core storage
        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []

        # result groups
        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        # internals for frame construction
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ----------------------------------------------------------------------
    # 1. POINT SAMPLING
    # ----------------------------------------------------------------------

    def sample_points(self):
        """Sample points on the curve or surface."""
        pts = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # CURVE MODE
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # SURFACE / BREp MODE
        else:
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

            pts.sort(key=lambda p: p.Z)

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ----------------------------------------------------------------------
    # 2. FRAMES FROM GEOMETRY
    # ----------------------------------------------------------------------

    def frames_from_geometry(self):
        """Construct compas Frames at each sampled point."""
        frames = []

        # CURVE MODE
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    x.unitize()
                    y.unitize()
                else:
                    # fallback tangent-based frame
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    x.unitize()
                    y = Vector(0, 0, 1).cross(x)
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
                    x.unitize()
                    y.unitize()
                else:
                    x = Vector(1, 0, 0)
                    y = Vector(0, 1, 0)

                frames.append(Frame(pt, x, y))

        else:
            # fallback
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ----------------------------------------------------------------------
    # 3. NEAREST NEIGHBOUR EDGES
    # ----------------------------------------------------------------------

    def frames_to_edges(self):
        """Find nearest neighbour edges and compute edge frames."""
        pts = [f.point for f in self.frames]
        n = len(pts)

        if n < 2:
            self.edges = []
            self.edge_frames = []
            self.edge_vectors = []
            return [], []

        edges = set()

        # NN search
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

        # Build frames along edges
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
    # Utility for L-rules
    # ----------------------------------------------------------------------

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        if isinstance(rule_str, (int, float)):
            return [rule_str]
        vals = []
        for tok in rule_str.split(","):
            tok = tok.strip()
            if tok == "":
                continue
            try:
                if "." in tok:
                    vals.append(float(tok))
                else:
                    vals.append(int(tok))
            except:
                pass
        return vals

    # ----------------------------------------------------------------------
    # 4. BRANCHING (L-style rules)
    # ----------------------------------------------------------------------

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # Build root sticks (IMPORTANT: pass edge frame as parent_frame)
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            s = Stick(axis,
                      length=self.stick_length,
                      width=self.stick_width,
                      depth=self.stick_depth,
                      parent_frame=f)
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # Branch from each root
        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            for k in range(int(steps)):
                fi = face_seq[k % len(face_seq)] if face_seq else 0
                ang = angle_seq[k % len(angle_seq)] if angle_seq else stick_angle
                B.grow_once(face_index=int(fi), stick_angle=float(ang))

            # skip root duplicate
            self.branch_sticks.extend(B.sticks[1:])

        return self.branch_sticks

    # ----------------------------------------------------------------------
    # 5. BRIDGING (optional)
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
                if sticks[i].intersects(sticks[j], clearance=clearance):
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
        clearance=0.0,
        face_rule=None,
        angle_rule=None,
        verbose=False,
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
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * (len(self.branch_sticks) + len(self.bridge_sticks))

        return self.branch_sticks + self.bridge_sticks
