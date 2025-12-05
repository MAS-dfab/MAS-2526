# rf_core.py
# RootFrames engine, COMPAS geometry only

import random
import Rhino.Geometry as rg  # type: ignore # only for sampling / normals

from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


class RootFrames(object):
    """
    RootFrames pipeline:
      1) Sample points on curve or surface
      2) Build 3D frames at samples (curve/surface aware)
      3) Build nearest-neighbour edges
      4) Grow sticks on edges (branching)
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

        self.stick_length = float(stick_length or Stick.DEFAULT_LEN)
        self.stick_width = float(stick_width or Stick.DEFAULT_SIZE)
        self.stick_depth = float(stick_depth or Stick.DEFAULT_SIZE)

        self.points = []
        self.frames = []
        self.edges = []
        self.edge_frames = []
        self.edge_vectors = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

    # ------------------------------------------------------------------ #
    # 1. SAMPLE POINTS                                                    #
    # ------------------------------------------------------------------ #
    def sample_points(self):
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

            for _ in range(self.point_density):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

        # SURFACE MODE
        else:
            brep = self.surface_input.ToBrep()
            face = brep.Faces[0]
            self._rg_face = face
            udom = face.Domain(0)
            vdom = face.Domain(1)

            for _ in range(self.point_density):
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                pts.append(p)
                self._uv_params.append((u, v))

        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------ #
    # 2. FRAMES FROM GEOMETRY                                            #
    # ------------------------------------------------------------------ #
    def frames_from_geometry(self):
        frames = []

        # CURVE MODE
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve
            for pt_c, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)
                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                else:
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    y = Vector(0, 0, 1).cross(x)
                x.unitize()
                y.unitize()
                frames.append(Frame(pt_c, x, y))

        # SURFACE MODE (robust, works on spheres)
        elif self._rg_face and self._uv_params:
            face = self._rg_face
            for pt_c, (u, v) in zip(self.points, self._uv_params):
                normal = face.NormalAt(u, v)
                z = Vector(normal.X, normal.Y, normal.Z)
                if z.length < 1e-6:
                    z = Vector(0, 0, 1)
                z.unitize()

                if abs(z.z) > 0.9:
                    helper = Vector(1, 0, 0)
                else:
                    helper = Vector(0, 0, 1)

                x = helper.cross(z)
                if x.length < 1e-6:
                    x = Vector(1, 0, 0)
                x.unitize()

                y = z.cross(x)
                if y.length < 1e-6:
                    y = Vector(0, 1, 0)
                y.unitize()

                frames.append(Frame(pt_c, x, y))

        else:
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ------------------------------------------------------------------ #
    # 3. NEAREST NEIGHBOUR EDGES                                         #
    # ------------------------------------------------------------------ #
    def frames_to_edges(self):
        pts = [f.point for f in self.frames]
        n = len(pts)
        if n < 2:
            return [], []

        edges = set()
        for i in range(n):
            p0 = pts[i]
            best = 1e9
            j_best = None
            for j in range(n):
                if i == j:
                    continue
                d = p0.distance_to_point(pts[j])
                if d < best:
                    best = d
                    j_best = j
            edges.add(tuple(sorted((i, j_best))))

        edges = list(edges)
        self.edges = edges

        eframes = []
        evectors = []
        for i, j in edges:
            p0 = pts[i]
            p1 = pts[j]
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

    # ------------------------------------------------------------------ #
    # helper: parse rule strings                                         #
    # ------------------------------------------------------------------ #
    def _parse_rule(self, s):
        if not s:
            return []
        if isinstance(s, (int, float)):
            return [float(s)]
        vals = []
        for tok in str(s).split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                vals.append(float(tok))
            except Exception:
                pass
        return vals

    # ------------------------------------------------------------------ #
    # 4. BRANCHING                                                       #
    # ------------------------------------------------------------------ #
    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None):

        self.root_sticks = []
        self.branch_sticks = []

        # root sticks
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)
            s = Stick(
                axis=axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f,
            )
            self.root_sticks.append(s)
            self.branch_sticks.append(s)

        faces = self._parse_rule(face_rule)
        angles = self._parse_rule(angle_rule)

        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            for k in range(int(steps)):
                fi = faces[k % len(faces)] if faces else 0.0
                ang = angles[k % len(angles)] if angles else stick_angle
                B.grow_once(face_index=int(fi), stick_angle=float(ang))

            self.branch_sticks.extend(B.sticks[1:])  # drop duplicate root

        return self.branch_sticks

    # ------------------------------------------------------------------ #
    # 5. BRIDGING                                                        #
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 6. COLLISION DETECTION                                             #
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # 7. RUN                                                              #
    # ------------------------------------------------------------------ #
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
            self.detect_collisions(clearance)
        else:
            self.collision_flags = [False] * (
                len(self.branch_sticks) + len(self.bridge_sticks)
            )

        return self.branch_sticks + self.bridge_sticks
