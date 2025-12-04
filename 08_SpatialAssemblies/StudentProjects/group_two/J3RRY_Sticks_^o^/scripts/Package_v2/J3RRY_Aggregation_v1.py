from compas.geometry import Transformation
from J3RRY_SingleStick_v1 import Stick
from J3RRY_Collision_v1 import Collision
import math, random


class Aggregation:
    def __init__(self, first_frame, length_pattern=[100], aggregation_type=0, global_seed=None):
        """
        Constructor for Stick Aggregation.
        
        Args:
            first_frame: Frame for the first stick.
            length: Length of each stick (defaults to 100.0)
            aggregation_type: 0 = regular, 1 = random
            global_seed: Seed for random generator (defaults to None)
        """
        self.sticks = []
        self.axes = []
        self.frames = []
        self.from_frames = []
        self.to_frames = []
        self.length_pattern = length_pattern  # can be a single value or list

        self.failed_sticks = []
        self.collision_log = []

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
        Private method to get the next length from the length pattern.
        
        Returns:
            float: next length value.
        """
        length = random.choice(self.length_pattern)

        return length

    def spawn_next_stick(self, angle=0, from_index=0, from_t=0.5, to_index=1, to_t=0.5):
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

        from_frame = current_stick.eval_frame(from_index, from_t)
        to_frame = current_stick.eval_frame(to_index, to_t)

        # Flip frame (fixed X axis)
        from_frame.yaxis = -from_frame.yaxis
        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        # Orient stick frame using frame to frame
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_frame = current_stick.frame.transformed(T)
        new_stick = Stick(new_frame, length=self._next_length())

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)
        self.frames.append(new_frame)
        # return [from_frame, to_frame, new_frame]


    def spawn_next_stick_random(self, angle=0, local_seed=None):
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
        self.spawn_next_stick(angle, from_index, from_t, to_index, to_t)


    def spawn_next_stick_random_with_rejection(self, angle=0, max_attempts=10, local_seed=None):
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
        params_from_index = []
        params_from_t = []
        params_to_index = []
        params_to_t = []


        current_stick = self.sticks[-1]
        for attempt in range(max_attempts):
        
            # Create random index and t value for the next stick
            from_index = random.randint(0, 3)
            from_t = random.random()
            to_index = random.randint(0, 3)
            to_t = random.random()

            params_from_index.append(from_index)
            params_from_t.append(from_t)
            params_to_index.append(to_index)
            params_to_t.append(to_t)

            from_frame = current_stick.eval_frame(from_index, from_t)
            to_frame = current_stick.eval_frame(to_index, to_t)

            # Flip frame (fixed X axis)
            from_frame.yaxis = -from_frame.yaxis
            # Rotate to_frame
            to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

            # Orient stick frame using frame to frame
            T = Transformation.from_frame_to_frame(from_frame, to_frame)
            new_frame = current_stick.frame.transformed(T)
            new_stick = Stick(new_frame, length=self._next_length())

            # --- collision checking
            collision_found = False
            for other in self.sticks[:-1]:    # skip parent
                if Collision(new_stick, other).check_collision():
                    collision_found = True
                    
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
                    "length": new_stick.length,  
                    "angle": angle
                }
            })

            # SUCCESS: append and return
            self.sticks.append(new_stick)
            self.axes.append(new_stick.axis)
            self.from_frames.append(from_frame)
            self.to_frames.append(to_frame)
            self.frames.append(new_frame)
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
                "length": new_stick.length,
                "angle": angle
            }
        })
        return None
    

    def spawn_next_stick_random_in_boundary_with_rejection(self, boundary, angle=0, max_attempts=10):
        

        return None


    def visualize(self):
        """
        Compute all stick geometries.
        
        Returns:
            List of Box geometries
        """
        return [stick.geometry for stick in self.sticks]
