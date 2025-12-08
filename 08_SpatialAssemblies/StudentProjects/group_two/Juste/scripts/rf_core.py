# rf_core.py
# RootFrames engine with:
#   - gradient sampling
#   - min-distance enforcement
#   - Y/Z family-based branching
#   - local alternation only when bridging needed
#   - 3D surface-aware frame reconstruction
#   - bridging (non-coplanar, limited distance)
#   - collision detection
#   - COMPAS-native Boxes for geometry

import random
import math

import Rhino.Geometry as rg # type: ignore
from compas.geometry import (
    Point,
    Vector,
    Line,
    Frame,
    Box,
    distance_point_point
)

from stick_fixed import Stick
from branch import BranchingModule
from bridge import BridgingModule


# ------------------------------------------------------------
# 1. Gradient-based spacing function
# ------------------------------------------------------------

def compute_local_spacing(z, zmin, zmax, dmax, dmin, dexp):
    if zmax == zmin:
        return dmin
    t = (z - zmin) / (zmax - zmin)
    return dmax - (dmax - dmin) * (t ** dexp)


# ------------------------------------------------------------
# 2. Poisson-like sampling on the surface
# ------------------------------------------------------------

def sample_surface_points(surface, count, dmax, dmin, dexp):
    """
    Poisson-like sampling on a surface with z-based spacing gradient.
    Returns:
        pts3d (world XYZ points)
        uv_params (param locations)
    """
    brep = surface.ToBrep()
    face = brep.Faces[0]

    # Compute bounding box along Z for gradient spacing
    bbox = brep.GetBoundingBox(True)
    zmin, zmax = bbox.Min.Z, bbox.Max.Z

    pts = []
    uv = []

    attempts = 0
    max_attempts = count * 50

    while len(pts) < count and attempts < max_attempts:
        attempts += 1

        # random UV
        u = random.random() * (face.Domain(0).T1 - face.Domain(0).T0) + face.Domain(0).T0
        v = random.random() * (face.Domain(1).T1 - face.Domain(1).T0) + face.Domain(1).T0

        p = face.PointAt(u, v)
        z = p.Z

        # spacing requirement
        dloc = compute_local_spacing(z, zmin, zmax, dmax, dmin, dexp)

        ok = True
        for q in pts:
            if distance_point_point(Point(p.X, p.Y, p.Z), q) < dloc:
                ok = False
                break

        if ok:
            pts.append(Point(p.X, p.Y, p.Z))
            uv.append((u, v))

    return pts, uv


# ------------------------------------------------------------
# 3. Frame generation (un-flattened)
# ------------------------------------------------------------

def compute_frame_from_surface(face, u, v):
    ok, plane = face.FrameAt(u, v)
    if not ok:
        # fallback
        return Frame(Point(0, 0, 0), Vector(1, 0, 0), Vector(0, 1, 0))

    origin = Point(plane.Origin.X, plane.Origin.Y, plane.Origin.Z)
    xvec = Vector(plane.XAxis.X, plane.XAxis.Y, plane.XAxis.Z)
    yvec = Vector(plane.YAxis.X, plane.YAxis.Y, plane.YAxis.Z)
    xvec.unitize()
    yvec.unitize()

    return Frame(origin, xvec, yvec)


# ------------------------------------------------------------
# 4. RootFrames Class
# ------------------------------------------------------------

class RootFrames:
    def __init__(
        self,
        surface=None,
        curve=None,
        point_density=20,
        stick_length=1000.0,
        stick_width=200.0,
        stick_depth=200.0,
        d_max=800.0,
        d_min=200.0,
        d_exp=1.5,
        bridge_threshold=600.0,
    ):
        self.surface_input = surface
        self.curve_input = curve

        self.point_density = point_density
        self.stick_length = stick_length
        self.stick_width = stick_width
        self.stick_depth = stick_depth

        self.d_max = d_max
        self.d_min = d_min
        self.d_exp = d_exp

        self.bridge_threshold = bridge_threshold

        self.points = []
        self.uv_params = []
        self.frames = []
        self.root_sticks = []
        self.branch_sticks = []
        self.bridge_sticks = []

        self.collision_flags = []

    # ------------------------------------------------------------
    # Sampling
    # ------------------------------------------------------------

    def sample_points(self):
        if self.surface_input is None:
            raise ValueError("Surface-only mode currently supported.")

        pts, uv = sample_surface_points(
            self.surface_input,
            self.point_density,
            self.d_max,
            self.d_min,
            self.d_exp,
        )
        self.points = pts
        self.uv_params = uv

    # ------------------------------------------------------------
    # Build frames
    # ------------------------------------------------------------

    def frames_from_surface(self):
        brep = self.surface_input.ToBrep()
        face = brep.Faces[0]

        frames = []
        for (u, v) in self.uv_params:
            f = compute_frame_from_surface(face, u, v)
            frames.append(f)

        self.frames = frames

    # ------------------------------------------------------------
    # Create initial root sticks
    # ------------------------------------------------------------

    def build_root_sticks(self):
        self.root_sticks = []

        for f in self.frames:
            origin = f.point
            tangent = f.xaxis.unitized()
            start = origin - tangent * (0.5 * self.stick_length)
            end   = origin + tangent * (0.5 * self.stick_length)
            axis = Line(start, end)

            s = Stick(
                axis,
                length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                parent_frame=f,
            )
            self.root_sticks.append(s)

    # ------------------------------------------------------------
    # Branching
    # ------------------------------------------------------------

    def grow_branching(self, steps=3, stick_angle=0.0):
        self.branch_sticks = []

        for root in self.root_sticks:
            B = BranchingModule(
                root_stick=root,
                stick_length=self.stick_length,
                width=self.stick_width,
                depth=self.stick_depth,
                offset01=0.5,
            )

            chain = B.grow_chain(steps, stick_angle)
            self.branch_sticks.extend(chain)

    # ------------------------------------------------------------
    # Bridging
    # ------------------------------------------------------------

    def grow_bridging(self):
        BM = BridgingModule(
            stick_list=self.branch_sticks[:],
            stick_length=self.stick_length,
            width=self.stick_width,
            depth=self.stick_depth,
            bridge_threshold=self.bridge_threshold,
            max_generations=3,
        )
        bridges = BM.build()
        self.bridge_sticks = bridges

    # ------------------------------------------------------------
    # Collision detection
    # ------------------------------------------------------------

    def detect_collisions(self):
        all_sticks = self.branch_sticks + self.bridge_sticks
        n = len(all_sticks)

        flags = [False] * n
        for i in range(n):
            for j in range(i + 1, n):
                if all_sticks[i].intersects(all_sticks[j], clearance=0.0):
                    flags[i] = True
                    flags[j] = True

        self.collision_flags = flags

    # ------------------------------------------------------------
    # RUN FULL PIPELINE
    # ------------------------------------------------------------

    def run(self, steps=3, stick_angle=0.0, do_bridging=True, detect_collisions=True):

        self.sample_points()
        self.frames_from_surface()
        self.build_root_sticks()

        # branching
        self.grow_branching(steps, stick_angle)

        # bridging
        if do_bridging:
            self.grow_bridging()

        # collision detection
        if detect_collisions:
            self.detect_collisions()

        return self.root_sticks, self.branch_sticks, self.bridge_sticks, self.frames
