# rf_core.py
# RootFrames core using COMPAS geometry + BranchingModule + BridgingModule.

import random
import Rhino.Geometry as rg  # type: ignore
from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


class RootFrames(object):
    """
    RootFrames engine:

      1) Density-aware sampling on curve or surface with vertical gradient.
      2) 3D frames from Rhino frames (tangent to surface/curve).
      3) Nearest-neighbour edges + edge frames.
      4) Branching phase (L-system rules, family-locked faces).
      5) Optional bridging between non-coplanar sticks (cross-family).
      6) Optional collision detection.

    Debug/data channels:
      - self.root_sticks
      - self.branch_sticks
      - self.bridge_sticks
      - self.collision_flags
      - self.frames
      - self.edge_frames
      - self.sample_points_list
      - self.sample_radii
    """

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=20,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
    ):
        self.surface_input = surface
        self.curve_input = curve

        self.point_density = int(point_density)

        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        self.points = []
        self.frames = []
        self.edge_frames = []
        self.edge_vectors = []
        self.edges = []

        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []
        self.collision_flags = []

        self.sample_points_list = []
        self.sample_radii = []
        self._rg_face = None
        self._uv_params = []
        self._rg_curve = None
        self._curve_t = []

        # gradient parameters (set in run)
        self.d_max = 1000.0
        self.d_min = 250.0
        self.d_exp = 1.5

        # bridging threshold (set in run)
        self.bridge_density_threshold = 1000.0

    # ------------------------------------------------------------------ #
    # Utility: gradient spacing                                          #
    # ------------------------------------------------------------------ #

    def _spacing_for_z(self, z, zmin, zmax):
        if zmax <= zmin + 1e-6:
            z_norm = 0.0
        else:
            z_norm = (z - zmin) / (zmax - zmin)
            z_norm = max(0.0, min(1.0, z_norm))
        return self.d_max - (self.d_max - self.d_min) * (z_norm ** self.d_exp)

    # ------------------------------------------------------------------ #
    # 1. POINT SAMPLING (gradient + Poisson-like rejection)              #
    # ------------------------------------------------------------------ #

    def sample_points(self):
        pts = []
        radii = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # CURVE MODE ------------------------------------------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = dom.T0, dom.T1

            bbox = crv.GetBoundingBox(True)
            zmin = bbox.Min.Z
            zmax = bbox.Max.Z

            max_tries = self.point_density * 20
            tries = 0

            while len(pts) < self.point_density and tries < max_tries:
                tries += 1
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                z = p.Z
                r = self._spacing_for_z(z, zmin, zmax)

                accept = True
                for q in pts:
                    if p.DistanceTo(q) < r:
                        accept = False
                        break

                if accept:
                    pts.append(p)
                    radii.append(r)
                    self._curve_t.append(t)

        # SURFACE / BREP MODE --------------------------------------------
        else:
            if self.surface_input is None:
                raise Exception("RootFrames.sample_points: no surface or curve input.")

            brep = self.surface_input.ToBrep()
            if not brep or brep.Faces.Count == 0:
                raise Exception("RootFrames.sample_points: Brep has no faces.")

            face = brep.Faces[0]
            self._rg_face = face

            udom = face.Domain(0)
            vdom = face.Domain(1)

            bbox = face.GetBoundingBox(True)
            zmin = bbox.Min.Z
            zmax = bbox.Max.Z

            max_tries = self.point_density * 40
            tries = 0

            while len(pts) < self.point_density and tries < max_tries:
                tries += 1
                u = random.uniform(udom.T0, udom.T1)
                v = random.uniform(vdom.T0, vdom.T1)
                p = face.PointAt(u, v)
                z = p.Z
                r = self._spacing_for_z(z, zmin, zmax)

                accept = True
                for q in pts:
                    if p.DistanceTo(q) < r:
                        accept = False
                        break

                if accept:
                    pts.append(p)
                    radii.append(r)
                    self._uv_params.append((u, v))

            pts.sort(key=lambda p: p.Z)

        self.sample_points_list = pts
        self.sample_radii = radii
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]
        return self.points

    # ------------------------------------------------------------------ #
    # 2. FRAMES FROM GEOMETRY                                           #
    # ------------------------------------------------------------------ #

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
                    x.unitize()
                    y.unitize()
                else:
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
            for pt in self.points:
                frames.append(Frame(pt, Vector(1, 0, 0), Vector(0, 1, 0)))

        self.frames = frames
        return frames

    # ------------------------------------------------------------------ #
    # 3. NEAREST NEIGHBOUR EDGES                                        #
    # ------------------------------------------------------------------ #

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
            if j_best is not None:
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
    # Utility for L-rules                                               #
    # ------------------------------------------------------------------ #

    def _parse_rule(self, rule_str):
        if not rule_str:
            return []
        if isinstance(rule_str, (int, float)):
            return [rule_str]
        vals = []
        for tok in str(rule_str).split(","):
            tok = tok.strip()
            if not tok:
                continue
            try:
                if "." in tok:
                    vals.append(float(tok))
                else:
                    vals.append(int(tok))
            except Exception:
                continue
        return vals

    # ------------------------------------------------------------------ #
    # 4. BRANCHING (families, OPTION B)                                 #
    # ------------------------------------------------------------------ #

    def grow_branching(self, steps, stick_angle, offset01, face_rule=None, angle_rule=None):
        self.root_sticks = []
        self.branch_sticks = []

        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # Build root sticks and their branch families
        for f, v in zip(self.edge_frames, self.edge_vectors):
            axis = Line(f.point, f.point + v * self.stick_length)

            # Randomly choose family for this root: 'Y' or 'Z'
            family = "Y" if random.random() < 0.5 else "Z"

            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f,
                generation=0,
                kind="root",
                family=family,
            )
            self.root_sticks.append(root)
            self.branch_sticks.append(root)

            # Branching module for this root
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
                family=family,
            )

            # family-specific default faces
            default_face = 2 if family == "Y" else 4

            for k in range(int(steps)):
                # face index from sequence (if any), then clamped in BranchingModule
                if face_seq:
                    fi = int(face_seq[k % len(face_seq)])
                else:
                    fi = default_face

                # angle from sequence or default
                if angle_seq:
                    ang = float(angle_seq[k % len(angle_seq)])
                else:
                    ang = float(stick_angle)

                B.grow_once(face_index=fi, stick_angle=ang)

            # skip root duplicate
            self.branch_sticks.extend(B.sticks[1:])

        return self.branch_sticks

    # ------------------------------------------------------------------ #
    # 5. BRIDGING (cross-family only)                                   #
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
            max_generation=3,
            angle_dot_max=0.75,
            distance_threshold=self.bridge_density_threshold,
        )
        self.bridge_sticks = BM.build()
        return self.bridge_sticks

    # ------------------------------------------------------------------ #
    # 6. COLLISION DETECTION                                            #
    # ------------------------------------------------------------------ #

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

    # ------------------------------------------------------------------ #
    # 7. RUN PIPELINE                                                   #
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
        d_max=1000.0,
        d_min=250.0,
        d_exp=1.5,
        bridge_density_threshold=1000.0,
    ):
        # store gradient params
        self.d_max = float(d_max)
        self.d_min = float(d_min)
        self.d_exp = float(d_exp)
        self.bridge_density_threshold = float(bridge_density_threshold)

        # 1–3
        self.sample_points()
        self.frames_from_geometry()
        self.frames_to_edges()

        # 4
        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
        )

        # 5
        if do_bridging:
            self.grow_bridging()
        else:
            self.bridge_sticks = []

        # 6
        if detect_collisions:
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * (
                len(self.branch_sticks) + len(self.bridge_sticks)
            )

        return self.branch_sticks + self.bridge_sticks
