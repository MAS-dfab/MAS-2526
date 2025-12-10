from compas.geometry import Plane, Box, Line, Vector, Frame, Rotation
from compas.geometry import Transformation, Translation
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
        self.my_modules = []
        self.layer_modules = []

        if first_module:
            self.my_modules.append(first_module)

    # ============================================================
    # 生成下一個 module
    # ============================================================
    def compute_next_module(self,
                            from_stick_index=0, 
                            from_face_index=0,  
                            from_t=0.5, 
                            to_stick_index=3,
                            to_face_index=3,    
                            to_t=0.5,   
                            angle=0):

        base_module = self.my_modules[-1]

        from_stick = base_module[from_stick_index]
        from_frame = from_stick.eval_frame(from_face_index, from_t, z_offset=True)
        self.from_frame = from_frame

        to_stick = base_module[to_stick_index]
        to_frame = to_stick.eval_frame(to_face_index, to_t)
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)
        self.to_frame = to_frame

        T = Transformation.from_frame_to_frame(to_frame, from_frame)

        new_module = []
        for stick in base_module:
            new_frame = stick.frame.transformed(T)
            new_stick = Stick(new_frame, stick.length)
            new_module.append(new_stick)

        self.my_modules.append(new_module)

    # ============================================================
    # 產生多層 layer（每層平移一次）
    # ============================================================
    def move_by_layer(self, translation_vector=(0, 0, 0), layer_iteration=1):
        t_vec = Vector(*translation_vector)

        for i in range(layer_iteration):
            T = Translation.from_vector(t_vec * (i + 1))

            layer = []
            for module in self.my_modules:
                new_module = []
                for stick in module:
                    new_stick = Stick(stick.frame.transformed(T), stick.length)
                    new_module.append(new_stick)
                layer.append(new_module)

            self.layer_modules.append(layer)

    # ============================================================
    # Stick objects（原始 + 所有 layers）
    # ============================================================
    def sticks_object(self):
        sticks = []

        # 原始 modules
        for module in self.my_modules:
            sticks.extend(module)

        # layers
        for layer in self.layer_modules:
            for module in layer:
                sticks.extend(module)

        return sticks

    # ============================================================
    # Geometry（回傳 compas Box）
    # ============================================================
    def visualize(self):
        geos = []

        for stick in self.sticks_object():
            geos.append(stick.geometry)

        return geos