from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import Transformation
from Sticks_Single_1 import Stick
from Sticks_Multiple_1 import TiltedModule
import math, random


class SingleStickAggregation:
    def __init__(self, first_frame, length=200, aggregation_type=0, global_seed=None):
        self.length = length
        self.sticks = []
        self.axes = []
        self.new_frames = []
        self.from_frames = []
        self.to_frames = []

        # 0 = regular, 1 = random
        self.aggregation_type = aggregation_type
        self.global_seed = global_seed

        # 建立獨立 random generator（不污染 global random）# rng:random generator (隨機數產生器)
        if isinstance(global_seed, int):
            self.rng = random.Random(global_seed) # 固定隨機，可重現
        else:
            self.rng = random.Random()  # 非固定隨機

        self._init_first_stick(first_frame)

    # 初始化第一根 Stick
    def _init_first_stick(self, first_frame):
        s = Stick(first_frame, self.length)
        self.sticks.append(s)
        self.axes.append(s.axis)

    # Regular generate（不使用 random）
    def compute_next_stick_regular(self, angle=0, from_index=0, from_t=0.5, to_index=1, to_t=0.5):

        current_stick = self.sticks[-1]

        from_frame = current_stick.eval_frame(from_index, from_t)
        to_frame = current_stick.eval_frame(to_index, to_t)

        # Flip frame (fixed X axis)
        from_frame.yaxis = -from_frame.yaxis
        from_frame.xaxis = from_frame.yaxis.cross(from_frame.zaxis)

        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        # Orient stick frame using frame-to-frame
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_frame = current_stick.frame.transformed(T)
        new_stick = Stick(new_frame, length=self.length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.new_frames.append(new_frame)
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)

    # Random generate（使用 self.rng）
    # aggregation_type = 1 時才會用到
    def compute_next_stick_random(self, angle=0, local_seed=None):

        current_stick = self.sticks[-1]

        # 若 aggregation_type=1 且 local_seed 提供 → 可以局部覆蓋 random 行為
        if self.aggregation_type == 1 and isinstance(local_seed, int):
            rng = random.Random(local_seed)
        else:
            rng = self.rng

        # Create random index and t values
        from_index = rng.randint(0, 3)
        from_t = rng.random()
        to_index = rng.randint(0, 3)
        to_t = rng.random()

        from_frame = current_stick.eval_frame(from_index, from_t)
        to_frame = current_stick.eval_frame(to_index, to_t)

        # Flip frame (fixed X axis)
        from_frame.yaxis = -from_frame.yaxis
        from_frame.xaxis = from_frame.yaxis.cross(from_frame.zaxis)

        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        # Orient stick frame
        T = Transformation.from_frame_to_frame(from_frame, to_frame)
        new_frame = current_stick.frame.transformed(T)
        new_stick = Stick(new_frame, length=self.length)

        # Collect Data
        self.sticks.append(new_stick)
        self.axes.append(new_stick.axis)
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)
        self.new_frames.append(new_frame)
        
    def compute_next_module(self, from_index=0, to_index=1, from_t=0.5, to_t=0.5, angle=0):
        """
        Create a NEW module (multiple sticks) from a previous module.
        from_index, to_index apply to previous module (multi-stick indexing)
        """
        previous_module = self.sticks[-4:]   # last 4 sticks are one module
        new_module_sticks = []

        for i in range(4):

            base_stick = previous_module[i]

            from_frame = base_stick.eval_frame(from_index, from_t)
            to_frame   = base_stick.eval_frame(to_index, to_t)

            # Flip frame
            from_frame.yaxis = -from_frame.yaxis
            from_frame.xaxis = from_frame.yaxis.cross(from_frame.zaxis)

            # Rotation
            to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

            # Frame mapping
            T = Transformation.from_frame_to_frame(from_frame, to_frame)
            new_frame = base_stick.frame.transformed(T)

            # Create new stick
            s = Stick(new_frame, length=self.length)
            new_module_sticks.append(s)

        # Save new module
        self.sticks.extend(new_module_sticks)


    # Visualization
    def visualize(self):
        return [stick.geometry for stick in self.sticks]

class ModuleAggregation:
    def __init__(self, first_module=None):
        self.modules = []        # [[stick1,stick2,stick3,stick4], ...]
        self.axes = []
        self.base_frames  = []
        self.from_frames = []
        self.to_frames   = []

        if first_module:
            self._init_first_module(first_module)

    # ---------------------------------------------------------
    # 初始化：把 TiltedModule 的 4 根 sticks 存入 modules[0]
    # ---------------------------------------------------------
    def _init_first_module(self, first_module):
        # first_module.sticks is list of 4 Sticks
        module_unit = first_module.sticks
        # if len(m) != 4:
        #     raise ValueError("First module must contain exactly 4 sticks.")
        
        self.modules.append(module_unit)
        self.axes.append([s.axis for s in module_unit])
        self.base_frames.append([s.frame for s in module_unit])

    # ---------------------------------------------------------
    # core function：從前一個 module 生成下一個 module
    # ---------------------------------------------------------
    def compute_next_module(self,
                            ref_stick_index=-1,
                            from_face_index=0, from_t=0.5,
                            to_face_index=2,   to_t=0.5,
                            angle=0):

        # Last module
        previous_module = self.modules[-1]

        # Choose reference stick
        ref_stick = previous_module[ref_stick_index]

        # 1. Draw from_frame and to_frame
        from_frame = ref_stick.eval_frame(from_face_index, from_t)
        to_frame   = ref_stick.eval_frame(to_face_index, to_t)

        # Flip frame
        from_frame.yaxis = -from_frame.yaxis
        from_frame.xaxis = from_frame.yaxis.cross(from_frame.zaxis)

        # Rotate to_frame
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        # Orient stick frame using frame-to-frame transform
        T = Transformation.from_frame_to_frame(from_frame, to_frame)

        # Collect data
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)

        # 2. Copy module（4 sticks），each stick apply Transformation(T)
        new_module = []

        for stick in previous_module:

            new_frame = stick.frame.transformed(T)
            s_length = stick.length

            new_stick = type(stick)(
                frame=new_frame,
                length=s_length,
                width=stick.width,
                depth=stick.depth
            )

            new_module.append(new_stick)
            self.base_frames.append(new_frame)

        # ----------------------------------------------------
        # 3. 存入 modules
        # ----------------------------------------------------
        self.modules.append(new_module)

    # ---------------------------------------------------------
    # 將所有 modules flatten 成 geometry list
    # ---------------------------------------------------------
    def visualize(self):
        geos = []
        for module in self.modules:
            for stick in module:
                geos.append(stick.geometry)
        return geos
