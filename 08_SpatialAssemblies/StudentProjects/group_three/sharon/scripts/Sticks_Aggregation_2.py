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
                            from_stick_index=0, 
                            from_face_index=0,  
                            from_t=0.5, 
                            to_stick_index=3,
                            to_face_index=3,    
                            to_t=0.5,   
                            angle=0):

        # =========================================================
        # 0. 取得上一模組（4 sticks）
        # =========================================================
        if not self.modules:
            raise ValueError("No module exists. Initialize first_module first.")
        
        prev_module = self.modules[-1]     # list of 4 sticks


        # =========================================================
        # 1. Compute FROM frame
        # =========================================================
        from_stick = prev_module[from_stick_index]
        from_frame = from_stick.eval_frame(face_index=from_face_index, t_value=from_t)
        from_frame.point += from_frame.yaxis * from_stick.depth / 2

        # flip orientation to keep consistency
        from_frame.yaxis = -from_frame.yaxis
        from_frame.xaxis = -from_frame.yaxis.cross(from_frame.zaxis)


        # =========================================================
        # 2. Compute TO frame (on the SAME previous module)
        # =========================================================
        to_stick = prev_module[to_stick_index]
        to_frame = to_stick.eval_frame(face_index=to_face_index, t_value=to_t)
        to_frame.point += to_frame.yaxis * to_stick.depth / 2

        # Apply local rotation (like GH Orient + rotate)
        to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

        # store for debugging
        self.from_frames.append(from_frame)
        self.to_frames.append(to_frame)


        # =========================================================
        # 3. Compute transform A → B
        # =========================================================
        T = Transformation.from_frame_to_frame(from_frame, to_frame)


        # =========================================================
        # 4. Transform entire module(4 sticks) → new module
        # =========================================================
        new_module = []
        for stick in prev_module:

            new_frame = stick.frame.transformed(T)
            new_stick = Stick(new_frame, stick.length)

            new_module.append(new_stick)

        # Push new module into list
        self.modules.append(new_module)

        # Also update axes and frames for record
        self.axes.append([s.axis for s in new_module])
        self.base_frames.append([s.frame for s in new_module])

        return new_module


    # def compute_next_module(self,
    #                         from_stick_index=0, 
    #                         from_face_index=0,  
    #                         from_t=0.5, 
    #                         to_stick_index=3,
    #                         to_face_index=3,    
    #                         to_t=0.5,   
    #                         angle=0):
        
    #     """ 
    #     1. module_unit = 前面的所有sticks為一組模矩單元
    #     將這組模組單元放入 my_modules (list of lists)
    #     之後就可以從my_modules中抽取前一組模組orient到指定位置
    #     """
    #     my_modules = []
    #     module_unit = self.modules
    #     my_modules.append([module_unit])
    #     from_stick = module_unit[from_stick_index]
    #     from_frame = from_stick.eval_frame(face_index=from_face_index, t_value=from_t)
    #     from_frame.point += from_stick.depth/2

    #     # flip frame to have consistent orientation
    #     from_frame.yaxis = -from_frame.yaxis
    #     from_frame.xaxis = -from_frame.yaxis.cross(from_frame.zaxis)

    #     """
    #     2. 從某個特定frame (from_frame) orient 到某個特定frame (to_frame) 
    #     """
    #     to_stick = module_unit[to_stick_index]
    #     to_frame = to_stick.eval_frame(face_index=to_face_index, t_value=to_t)
    #     to_frame.point += to_stick.depth/2

    #     # rotate new module
    #     to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

    #     # store frames
    #     self.from_frames.append(from_frame)
    #     self.to_frames.append(to_frame)

    #     """
    #     3. compute transformation
    #     類似 GH中的orient, 將一組物件從 planeA orient 到 planeB 
    #     """
    #     T = Transformation.from_frame_to_frame(from_frame, to_frame)
        
    #     """
    #     4. copy module_unit(4 sticks), orient to new frame, put them in my_modules(list of lists)
    #     """
    #     for stick in module_unit:
    #         new_frame = stick.frame.transformed(T)
    #         new_length = stick.length
    #         new_stick = Stick(new_frame, new_length)

    #         my_modules.append([new_stick])
    #         self.axes.append([s.axis for s in my_modules[-1]]) #將上一組module的axis加到my_axes
    #         self.base_frames.append()
    #         self.from_frames.append()
    #         self.to_frames.append(new_frame)

    # def compute_next_module(self,
    #                         from_stick_index=0,
    #                         from_face_index=0,
    #                         from_t=0.5,
    #                         to_stick_index=0,
    #                         to_face_index=2,
    #                         to_t=0.5,
    #                         angle=0):

    #     # ---------------------------------------------
    #     # 1. reference: 來源 from_frame
    #     # ---------------------------------------------
    #     previous_module = self.modules[-1]
    #     ref_stick = previous_module[from_stick_index]

    #     from_frame = ref_stick.eval_frame(from_face_index, from_t)

    #     # flip frame to have consistent orientation
    #     from_frame.yaxis = -from_frame.yaxis
    #     from_frame.xaxis = from_frame.yaxis.cross(from_frame.zaxis)

    #     # ---------------------------------------------
    #     # 2. target: 目標 to_frame（來自某個 module）
    #     # ---------------------------------------------
    #     target_module = self.modules[-1]
    #     target_stick  = target_module[to_stick_index]

    #     to_frame = target_stick.eval_frame(to_face_index, to_t)

    #     # rotate new module (user control)
    #     to_frame.rotate(math.radians(angle), to_frame.zaxis, to_frame.point)

    #     # store frames
    #     self.from_frames.append(from_frame)
    #     self.to_frames.append(to_frame)

    #     # ---------------------------------------------
    #     # 3. Compute transformation
    #     # ---------------------------------------------
    #     T = Transformation.from_frame_to_frame(from_frame, to_frame)

    #     # ---------------------------------------------
    #     # 4. Copy 4 sticks → new module
    #     # ---------------------------------------------
    #     new_module = []
    #     for stick in previous_module:

    #         new_frame = stick.frame.transformed(T)

    #         new_stick = type(stick)(
    #             frame=new_frame,
    #             length=stick.length,
    #             width=stick.width,
    #             depth=stick.depth
    #         )

    #         new_module.append(new_stick)
    #         self.base_frames.append(new_frame)

    #     # push module
    #     self.modules.append(new_module)


    # ---------------------------------------------------------
    # 將所有 modules flatten 成 geometry list
    # ---------------------------------------------------------
    def visualize(self):
        geos = []
        for module in self.modules:
            for stick in module:
                geos.append(stick.geometry)
        return geos
