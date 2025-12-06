from compas_rhino.conversions import mesh_to_compas
from compas.geometry import Polyhedron, is_point_in_polyhedron
from J3RRY_SingleStick_v1 import Stick
from J3RRY_Collision_v1 import Collision
import math, random


class Aggregation:
    def __init__(self, first_frame, length_pattern=[100], angle_pattern=[0], aggregation_type=0, global_seed=None):
        """
        Constructor for Stick Aggregation.
        
        Args:
            first_frame: Frame for the first stick.
            length_pattern: List of lengths or single length value for sticks.
            aggregation_type: 0 = regular, 1 = random
            global_seed: Seed for random generator (defaults to None)
        """
        self.length_pattern = length_pattern
        self.angle_pattern = angle_pattern
        self.sticks = []
        self.axes = []
        self.frames = []

        self.failed_sticks = []
        self.valid_sticks = []
        self.scores = []
        self.collision_log = []

        self._boundary_polyhedron = None

        # 0 = regular, 1 = random
        self.aggregation_type = aggregation_type
        self.global_seed = global_seed
        if global_seed is not None:
            random.seed(global_seed)

        self._init_first_stick(first_frame)


    def _init_first_stick(self, first_frame):
        """
        Private method for creating the first stick.
        
        Args:
            first_frame: Frame for the first stick.
        """
        first_stick = Stick(first_frame, self._next_length())
        self.sticks.append(first_stick)
        self.axes.append(first_stick.axis)
        self.frames.append(first_frame)

    
    def _next_length(self):
        """
        Private method to get the next length from the list of length pattern.
        
        Returns:
            float: next length value.
        """
        return random.choice(self.length_pattern)
    

    def _next_angle(self):
        """
        Private method to get the next angle from the list of angle pattern.
        
        Returns:
            float: next angle in degrees.
        """
        return random.choice(self.angle_pattern)


    def _remap_t_with_margin(self, t, length, margin=None):
        """
        Private method to remap t value with margin.

        Args:
            t: float, original t value (0.0 to 1.0).
            margin: float, margin to apply (defaults to stick width).

        Returns:
            float: remapped t value within margin constraints.
        """
        if margin is None:
            margin = self.width
        
        # Avoid 2 * margin > length
        max_margin = length * 0.5 * 0.999
        m = min(margin, max_margin)

        t = max(0.0 ,min(1.0, t))

        u_min = m / length
        u_max = 1.0 - (m / length)
        u = u_min + t * (u_max - u_min)
        return u
    

    def _center_to_existing(self, candidate):
        """
        Private method to find the smallest distance(not squared) between the center point of a candidate stick to all existing sticks, excluding the last stick(parent).
        
        Args:
            candidate: type Stick, candidate stick.
        
        Returns:
            float: smallest distance(not squared) between candidate center to existing sticks, excluding the last stick(parent).
        """
        if not self.sticks:
            return float('inf')
        
        min_dist = float('inf')
        cx, cy, cz = candidate.midframe.point
        for s in self.sticks[:-1]:
            px, py, pz = s.midframe.point
            dx = cx - px
            dy = cy - py
            dz = cz - pz
            dist = dx*dx + dy*dy + dz*dz

            if dist < min_dist:
                min_dist = dist
        # return math.sqrt(min_dist)
        return min_dist
    

    def _set_boundary_polyhedron(self, mesh):
        """
        Private method to convert Rhino Mesh to COMPAS Polyhedron for boundary checking, and store it internally.
        
        Args:
            mesh: type Rhino Mesh, must be closed mesh. Will convert to COMPAS Polyhedron.
            
        Returns:
            type Polyhedron: COMPAS Polyhedron.
        """
        if mesh is None:
            return None
        if self._boundary_polyhedron is None:
            compas_mesh = mesh_to_compas(mesh)
            vertices, faces = compas_mesh.to_vertices_and_faces()
            self._boundary_polyhedron = Polyhedron(vertices, faces)
    

    def point_in_mesh(self, point, mesh):
        """
        Check if a point is inside a closed mesh boundary.

        Args:
            point: type Point, point to check.
            mesh: type Rhino Mesh, must be closed mesh. Will convert to COMPAS Polyhedron.

        Returns:/
            bool: True if point is inside mesh.
        """
        self._set_boundary_polyhedron(mesh)  # Lazy init
        if self._boundary_polyhedron is None:
            return True  # No boundary defined, view all points as inside
        
        return is_point_in_polyhedron(point, self._boundary_polyhedron)


    def spawn_next_stick(self, from_index=0, from_t=0.5, to_index=1, to_t=0.5):
        """
        Spawns next stick based on specified face index and t value.
        
        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            from_index: Face index (0-3) on the current stick to attach from.
            from_t: Relative position along the from-face (0.0 to 1.0).
            to_index: Face index (0-3) on the current stick to attach to.
            to_t: Relative position along the to-face (0.0 to 1.0).
        """
        current_stick = self.sticks[-1]
        new_length = self._next_length()
        angle = self._next_angle()

        # Apply margin to from_t
        safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
        # Evlaute frames for next stick
        from_frame = current_stick.eval_frame(from_index, safe_from_t)
        dir_vec = from_frame.zaxis
        dir_vec.unitize()
        next_frame = from_frame.copy()
        # Move next_frame along normal by half current stick depth
        next_frame.point += dir_vec * (current_stick.depth / 2)
        
        # Define which face relative to new stick attaches to previous stick
        which_face = to_index * 90
        next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

        # Rotate next_frame around normal
        if angle != 0:
            next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

        # Apply margin to to_t
        safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)
        # Adjust start point along negative x-axis by new stick length
        offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
        next_frame.point -= offset

        # Create new stick
        new_stick = Stick(next_frame, length=new_length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.frames.append(next_frame)


    def spawn_next_stick_random(self, local_seed=None):
        """
        Spawns next stick based on random face index and t value.

        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            local_seed: Seed for random generator (overrides global seed if provided).
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Create random index and t value for the next stick
        from_index = random.randint(0, 3)
        from_t = random.random()
        to_index = random.randint(0, 3)
        to_t = random.random()
        self.spawn_next_stick(from_index, from_t, to_index, to_t)


    def spawn_next_stick_random_with_rejection(self, max_attempts=10, local_seed=None):
        """
        Spawns next stick based on random face index and t value with collision rejection sampling.

        Args:
            angle: Angle to rotate the to_frame around its z-axis (in degrees).
            max_attempts: Maximum number of attempts to find a non-colliding position.
            local_seed: Seed for random generator (overrides global seed if provided).

        Records:
            - collision_log: pure metadata
            - failed_sticks: list of Stick objects colliding in each spawn attempt
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Preparation for logging
        failed_candidates = []
        params_length = []
        params_angle = []
        params_from_index = []
        params_from_t = []
        params_to_index = []
        params_to_t = []


        current_stick = self.sticks[-1]
        for attempt in range(max_attempts):
            new_length = self._next_length()
            angle = self._next_angle()
            # Create random index and t value for the next stick
            from_index = random.randint(0, 3)
            # Apply margin to from_t
            from_t = random.random()
            safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
            to_index = random.randint(0, 3)
            # Apply margin to to_t
            to_t = random.random()
            safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)

            # Collect Data for logging
            params_from_index.append(from_index)
            params_from_t.append(safe_from_t)
            params_to_index.append(to_index)
            params_to_t.append(safe_to_t)
            params_length.append(new_length)
            params_angle.append(angle)

            # Evaluate frames for next stick
            from_frame = current_stick.eval_frame(from_index, safe_from_t)
            dir_vec = from_frame.zaxis
            dir_vec.unitize()
            next_frame = from_frame.copy()
            # Move next_frame along normal by half current stick depth
            next_frame.point += dir_vec * (current_stick.depth / 2)
            
            # Define which face of new stick attaches to previous stick
            which_face = to_index * 90
            next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)

            # Rotate next_frame around normal
            if angle != 0:
                next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

            # Adjust start point along negative x-axis by new stick length
            offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
            next_frame.point -= offset

            # Create new stick
            new_stick = Stick(next_frame, length=new_length)

            # --- collision checking
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    break
                    
            if collision_found:
                failed_candidates.append(new_stick)
                continue  # try again
            self.failed_sticks.append(failed_candidates)

            self.collision_log.append({
                "attempts": attempt + 1,
                "fail_count": len(failed_candidates),
                "success": True,
                "reason": "No Collision",
                "params": {
                    "from_index": params_from_index,
                    "from_t": params_from_t,
                    "to_index": params_to_index,
                    "to_t": params_to_t,
                    "length": params_length,  
                    "angle": params_angle
                }
            })

            # SUCCESS: append and return
            self.sticks.append(new_stick)
            self.axes.append(new_stick.axis)
            self.frames.append(next_frame)
            return new_stick
        
        # If all attempts fail
        self.failed_sticks.append(failed_candidates)
        self.collision_log.append({
            "attempts": max_attempts,
            "fail_count": len(failed_candidates),
            "success": False,
            "reason": "Max Attempts Reached",
            "params": {
                "from_index": params_from_index,
                "from_t": params_from_t,
                "to_index": params_to_index,
                "to_t": params_to_t,
                "length": params_length,
                "angle": params_angle
            }
        })
        return None


    def spawn_next_stick_random_in_boundary(self, boundary=None, max_attempts=10, local_seed=None):
        """
        Spawns next stick based on random face index and t value with collision rejection sampling.

        Args:
            boundary: type Rhino Mesh, to constrain sticks within boundary.
            max_attempts: Maximum number of attempts to find a non-colliding position.
            local_seed: Seed for random generator (overrides global seed if provided).

        Records:
            - collision_log: pure metadata
            - failed_sticks: list of Stick objects colliding in each spawn attempt
        """
        # 0 = regular, 1 = random
        if self.aggregation_type == 0 and local_seed is not None:
            random.seed(local_seed)

        # Preparation for selecting candidate
        valid_candidates = []
        valid_scores = []

        # Preparation for logging
        failed_candidates = []
        params_length = []
        params_angle = []
        params_from_index = []
        params_from_t = []
        params_to_index = []
        params_to_t = []


        current_stick = self.sticks[-1]
        for attempt in range(max_attempts):
            new_length = self._next_length()
            angle = self._next_angle()
            # Create random index and t value for the next stick
            from_index = random.randint(0, 3)
            # Apply margin to from_t
            from_t = random.random()
            safe_from_t = self._remap_t_with_margin(from_t, length=current_stick.length, margin=current_stick.width / 2)
            to_index = random.randint(0, 3)
            # Apply margin to to_t
            to_t = random.random()
            safe_to_t = self._remap_t_with_margin(to_t, length=new_length, margin=current_stick.width / 2)
            
            # Collect Data for logging
            params_from_index.append(from_index)
            params_from_t.append(safe_from_t)
            params_to_index.append(to_index)
            params_to_t.append(safe_to_t)
            params_length.append(new_length)
            params_angle.append(angle)
            
            # Evaluate frames for next stick
            from_frame = current_stick.eval_frame(from_index, safe_from_t)
            dir_vec = from_frame.zaxis
            dir_vec.unitize()
            next_frame = from_frame.copy()
            # Move next_frame along normal by half current stick depth
            next_frame.point += dir_vec * (current_stick.depth / 2)
            
            # Define which face of new stick attaches to previous stick
            which_face = to_index * 90
            next_frame.rotate(math.radians(which_face), next_frame.xaxis, next_frame.point)
            # Rotate next_frame around normal
            if angle != 0:
                next_frame.rotate(math.radians(angle), dir_vec, next_frame.point)

            # Adjust start point along negative x-axis by new stick length
            offset = next_frame.xaxis.unitized() * (safe_to_t * new_length)
            next_frame.point -= offset
            # Create new stick
            new_stick = Stick(next_frame, length=new_length)

            ### ------- boundary checking -------
            if boundary is not None:
                is_inside = self.point_in_mesh(new_stick.midframe.point, boundary)
                if not is_inside:
                    failed_candidates.append(new_stick)
                    continue
            ### ------- collision checking -------
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    break
                    
            if collision_found:
                failed_candidates.append(new_stick)
            else:
                valid_candidates.append(new_stick)
                valid_scores.append(self._center_to_existing(new_stick))
        self.failed_sticks.append(failed_candidates)
        self.valid_sticks.append(valid_candidates)
        self.scores.append(valid_scores)

        # Select best candidate based on score
        if not valid_candidates:
            return None
        best_idx = max(range(len(valid_scores)), key=lambda i: valid_scores[i])
        best_candidate = valid_candidates[best_idx]

        # SUCCESS: append and return
        self.sticks.append(best_candidate)
        self.axes.append(best_candidate.axis)
        self.frames.append(best_candidate.frame)
        return best_candidate

        
    def visualize(self):
        """
        Compute all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
