# rf_core.py
# RootFrames core engine (COMPAS-based, density-aware)
# r: compas>=2.14.1

import random
import Rhino.Geometry as rg  # type: ignore

from compas.geometry import Point, Vector, Line, Frame

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


# ----------------------------------------------------------------------
# Utility: collision helper
# ----------------------------------------------------------------------

def sticks_intersect_any(candidate, stick_list, clearance=0.0):
    for s in stick_list:
        if candidate.intersects(s, clearance=clearance):
            return True
    return False


class RootFrames:
    """
    RootFrames engine:

      1) Sample points on a curve or surface
      2) Build 3D frames using Rhino's native frames
      3) Build nearest-neighbour edges & edge-frames
      4) Branching phase (L-system style rules, density-aware)
      5) Optional bridging (distance-based, density-friendly)
      6) Optional collision detection via Stick AABBs

    Debug/data channels:
      - self.root_sticks
      - self.branch_sticks
      - self.bridge_sticks
      - self.collision_flags
      - self.frames
      - self.edge_frames
      - self.nn_distances  (nearest-neighbour distance per anchor point)
    """

    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=50,
        stick_length=None,
        stick_width=None,
        stick_depth=None,
        # density field parameters (model units)
        d_max=None,     # spacing at bottom (largest)
        d_min=None,     # spacing at top   (smallest)
        d_exp=None,     # exponent for smooth gradient
        bridge_density_threshold=None,  # used as reference scale
    ):
        # geometry inputs
        self.surface_input = surface
        self.curve_input = curve

        # sampling density (target #points, not exact)
        self.point_density = int(point_density)

        # stick dimensions
        self.stick_length = stick_length or Stick.DEFAULT_LEN
        self.stick_width = stick_width or Stick.DEFAULT_SIZE
        self.stick_depth = stick_depth or Stick.DEFAULT_SIZE

        # density field parameters (you can override from GH)
        self.d_max = float(d_max) if d_max is not None else 2000.0  # bottom spacing
        self.d_min = float(d_min) if d_min is not None else 150.0   # top spacing
        self.d_exp = float(d_exp) if d_exp is not None else 1.5     # gradient shape

        # reference scale for “sparse vs dense”
        self.bridge_density_threshold = (
            float(bridge_density_threshold) if bridge_density_threshold is not None else 400.0
        )

        # core storage
        self.points = []          # [compas Point]
        self.frames = []          # [compas Frame]
        self.edge_frames = []     # [compas Frame]
        self.edge_vectors = []    # [compas Vector]
        self.edges = []           # [(i, j)]
        self.nn_distances = []    # [float], size = len(points)

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

    # ------------------------------------------------------------------
    # 1. POINT SAMPLING (curve or surface)
    # ------------------------------------------------------------------

    def _compute_nn_distances(self):
        """Nearest-neighbour distance for each point in self.points."""
        n = len(self.points)
        if n == 0:
            self.nn_distances = []
            return

        if n == 1:
            self.nn_distances = [0.0]
            return

        dists = []
        for i in range(n):
            pi = self.points[i]
            best = 1e9
            for j in range(n):
                if i == j:
                    continue
                d = pi.distance_to_point(self.points[j])
                if d < best:
                    best = d
            dists.append(best)
        self.nn_distances = dists

    def sample_points(self):
        """Sample points on curve/surface with a smooth density field in Z."""
        pts = []
        self._uv_params = []
        self._curve_t = []
        self._rg_face = None
        self._rg_curve = None

        # --------------------------------------------------------------
        # Curve mode
        # --------------------------------------------------------------
        if self.curve_input is not None and self.surface_input is None:
            crv = self.curve_input
            self._rg_curve = crv

            dom = crv.Domain
            t0, t1 = float(dom.T0), float(dom.T1)

            for _ in range(max(1, self.point_density)):
                t = random.uniform(t0, t1)
                p = crv.PointAt(t)
                pts.append(p)
                self._curve_t.append(t)

            # convert to COMPAS
            self.points = [Point(p.X, p.Y, p.Z) for p in pts]
            self._compute_nn_distances()
            return self.points

        # --------------------------------------------------------------
        # Surface / Brep mode with continuous density in Z
        # --------------------------------------------------------------
        if self.surface_input is None:
            raise RuntimeError("RootFrames.sample_points: no surface_input or curve_input.")

        brep = self.surface_input.ToBrep()
        if not brep or brep.Faces.Count == 0:
            raise RuntimeError("RootFrames.sample_points: Brep has no faces.")

        face = brep.Faces[0]
        self._rg_face = face

        udom = face.Domain(0)
        vdom = face.Domain(1)

        bbox = face.GetBoundingBox(True)
        Zmin = bbox.Min.Z
        Zmax = bbox.Max.Z

        # continuous spacing function (bottom sparse -> top dense)
        def min_distance_for_z(z):
            t = (z - Zmin) / max(1e-6, (Zmax - Zmin))
            t = max(0.0, min(1.0, t))
            return self.d_min + (self.d_max - self.d_min) * (1 - t) ** self.d_exp

        def ok(p, existing):
            mind = min_distance_for_z(p.Z)
            for q in existing:
                if p.DistanceTo(q) < mind:
                    return False
            return True

        accepted = []
        attempts = self.point_density * 400  # rejection sampler budget

        for _ in range(attempts):
            u = random.uniform(float(udom.T0), float(udom.T1))
            v = random.uniform(float(vdom.T0), float(vdom.T1))

            p = face.PointAt(u, v)

            if ok(p, accepted):
                accepted.append(p)
                self._uv_params.append((u, v))

            if len(accepted) >= self.point_density:
                break

        pts = accepted

        # convert to COMPAS
        self.points = [Point(p.X, p.Y, p.Z) for p in pts]

        # replace uv_params to match final points count
        if len(self._uv_params) > len(pts):
            self._uv_params = self._uv_params[: len(pts)]

        self._compute_nn_distances()
        return self.points

    # ------------------------------------------------------------------
    # 2. FRAMES FROM GEOMETRY
    # ------------------------------------------------------------------

    def frames_from_geometry(self):
        """Construct compas Frames at each sampled point."""
        frames = []

        # Curve mode
        if self._rg_curve and self._curve_t:
            crv = self._rg_curve

            for pt, t in zip(self.points, self._curve_t):
                ok, plane = crv.FrameAt(t)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    if x.length < 1e-6:
                        x = Vector(1, 0, 0)
                    else:
                        x.unitize()
                    if y.length < 1e-6:
                        y = Vector(0, 1, 0)
                    else:
                        y.unitize()
                else:
                    tan = crv.TangentAt(t)
                    x = Vector(tan.X, tan.Y, tan.Z)
                    if x.length < 1e-6:
                        x = Vector(1, 0, 0)
                    else:
                        x.unitize()
                    y = Vector(0, 0, 1).cross(x)
                    if y.length < 1e-6:
                        y = Vector(0, 1, 0)
                    y.unitize()

                frames.append(Frame(pt, x, y))

        # Surface mode
        elif self._rg_face and self._uv_params:
            face = self._rg_face

            for pt, (u, v) in zip(self.points, self._uv_params):
                ok, plane = face.FrameAt(u, v)

                if ok:
                    x = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
                    y = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
                    if x.length < 1e-6:
                        x = Vector(1, 0, 0)
                    else:
                        x.unitize()
                    if y.length < 1e-6:
                        y = Vector(0, 1, 0)
                    else:
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

    # ------------------------------------------------------------------
    # 3. NEAREST-NEIGHBOUR EDGES
    # ------------------------------------------------------------------

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

        # simple nearest neighbour per point
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

    # ------------------------------------------------------------------
    # Utility for L-rules
    # ------------------------------------------------------------------

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
                pass
        return vals

    def _angle_scale_from_density(self, local_density):
        """
        Density -> angle scaling factor in [0.3, 1.0].

        - If local_density is large (sparse), angle is reduced
        - If local_density is small (dense), angle is near the base
        """
        if self.bridge_density_threshold <= 0:
            return 1.0
        ratio = self.bridge_density_threshold / max(local_density, 1e-6)
        # sparse => ratio < 1 (angle shrinks), dense => ratio > 1 (clamped)
        return max(0.3, min(1.0, ratio))

    # ------------------------------------------------------------------
    # 4. BRANCHING (L-style rules, density-aware, collision aware)
    # ------------------------------------------------------------------

    def grow_branching(self, steps, stick_angle, offset01,
                       face_rule=None, angle_rule=None,
                       clearance_for_growth=0.0):

        self.root_sticks = []
        self.branch_sticks = []

        all_sticks = []

        # Build root sticks along edges
        for (i, j), edge_frame in zip(self.edges, self.edge_frames):
            f = edge_frame
            p0 = f.point
            p1 = self.frames[j].point

            v = Vector.from_start_end(p0, p1)
            if v.length < 1e-6:
                continue
            v.unitize()

            axis = Line(p0, p0 + v * self.stick_length)

            root = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f
            )

            self.root_sticks.append(root)
            self.branch_sticks.append(root)
            all_sticks.append(root)

        # rules
        face_seq = self._parse_rule(face_rule)
        angle_seq = self._parse_rule(angle_rule)

        # Branch from each root
        for root in self.root_sticks:
            # Approximate density at this root using its start point
            root_pt = root.axis.start
            # find nearest anchor point
            if self.points:
                best = 1e9
                best_d = self.bridge_density_threshold
                for p, d in zip(self.points, self.nn_distances):
                    dd = p.distance_to_point(root_pt)
                    if dd < best:
                        best = dd
                        best_d = d
                local_density = best_d
            else:
                local_density = self.bridge_density_threshold

            angle_scale = self._angle_scale_from_density(local_density)

            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=offset01,
            )

            for k in range(int(steps)):
                fi = int(face_seq[k % len(face_seq)]) if face_seq else 0

                if angle_seq:
                    base_ang = float(angle_seq[k % len(angle_seq)])
                else:
                    base_ang = float(stick_angle)

                ang = base_ang * angle_scale

                # tentatively grow child
                B.grow_once(face_index=fi, stick_angle=ang)
                new_child = B.sticks[-1]

                # collision check against all accepted sticks
                if sticks_intersect_any(new_child, all_sticks, clearance=clearance_for_growth):
                    # reject this child
                    B.sticks.pop()
                    continue

                # accept: add globally
                self.branch_sticks.append(new_child)
                all_sticks.append(new_child)

        return self.branch_sticks

    # ------------------------------------------------------------------
    # 5. BRIDGING (distance-based, density-friendly)
    # ------------------------------------------------------------------

    def grow_bridging(self, max_bridge_distance=None, min_angle_deg=15.0):
        """Connect nearby sticks via BridgingModule.

        max_bridge_distance : if None, derived from density threshold.
        """
        if not self.branch_sticks:
            self.bridge_sticks = []
            return []

        if max_bridge_distance is None:
            # rough default: 1.2x density threshold or 1.5x stick length
            max_bridge_distance = max(
                1.2 * self.bridge_density_threshold,
                1.5 * self.stick_length,
            )

        BM = BridgingModule(
            stick_list=self.branch_sticks,
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
            max_distance=max_bridge_distance,
            min_angle_deg=min_angle_deg,
        )

        self.bridge_sticks = BM.build()
        return self.bridge_sticks

    # ------------------------------------------------------------------
    # 6. COLLISION DETECTION
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # 7. RUN PIPELINE
    # ------------------------------------------------------------------

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
        if verbose:
            print("RootFrames.run: sampling points...")
        self.sample_points()

        if verbose:
            print("RootFrames.run: building frames...")
        self.frames_from_geometry()

        if verbose:
            print("RootFrames.run: building edges...")
        self.frames_to_edges()

        if verbose:
            print("RootFrames.run: growing branching...")
        self.grow_branching(
            steps=steps,
            stick_angle=stick_angle,
            offset01=offset01,
            face_rule=face_rule,
            angle_rule=angle_rule,
            clearance_for_growth=clearance,
        )

        if do_bridging:
            if verbose:
                print("RootFrames.run: growing bridges...")
            self.grow_bridging()
        else:
            self.bridge_sticks = []

        if detect_collisions:
            if verbose:
                print("RootFrames.run: detecting collisions...")
            self.detect_collisions(clearance=clearance)
        else:
            self.collision_flags = [False] * (len(self.branch_sticks) + len(self.bridge_sticks))

        if verbose:
            print("RootFrames.run: done.")

        return self.branch_sticks + self.bridge_sticks
