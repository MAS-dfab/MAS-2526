from compas.geometry import Frame, Vector, Line, Box, bounding_box, Translation, Point


class stick_from_frame:
    """A stick defined by a center frame."""
    
    SIZE = 13.0
    WIDTH = SIZE
    DEPTH = SIZE

    def __init__(self, center_frame, stick_length, width=None, depth=None):
        self.center_frame = center_frame
        self.length = stick_length
        self.width = width or stick_from_frame.WIDTH
        self.depth = depth or stick_from_frame.DEPTH
        
        half_length = self.length / 2
        self.axis = Line(
            center_frame.point - center_frame.xaxis * half_length,
            center_frame.point + center_frame.xaxis * half_length
        )
    
    @property
    def start_point(self):
        return self.axis.start
    
    @property
    def end_point(self):
        return self.axis.end
    
    @property
    def direction(self):
        return self.center_frame.xaxis
    
    @property
    def geometry(self):
        return Box(self.length, self.width, self.depth, self.center_frame)
    
    @property
    def frame(self):
        return self.center_frame


class CubeModule:
    def __init__(self, root_frame, stick_length=200, width=None, depth=None):
        """
        Creates a cube frame from sticks using stick_from_frame.
        Sticks interlock at corners without intersecting.
        """
        self.root_frame = root_frame
        self.stick_length = stick_length
        self.depth = depth or stick_from_frame.DEPTH  
        self.width = width or stick_from_frame.WIDTH 
        self.sticks = []
        self._init_sticks(root_frame)
    
    def _create_stick_from_start(self, start_point, direction, up_hint, length):
        """Create stick given its START point (not center)."""
        xaxis = Vector(*direction)
        xaxis.unitize()
        
        zaxis = xaxis.cross(up_hint)
        zaxis.unitize()
        yaxis = zaxis.cross(xaxis)
        yaxis.unitize()
        
        center_point = start_point + xaxis * (length / 2)
        center_frame = Frame(center_point, xaxis, yaxis)
        return stick_from_frame(center_frame, length, width=self.width, depth=self.depth)
    
    def _build_cube_sticks(self, frame, stick_length):
        """
        Build 12 sticks for a cube matching the original interlocking pattern.
        """
        sticks = []
        
        stick_vert_length = stick_length - (self.width * 2) + self.depth
        half_depth = self.depth / 2
        half_width = self.width / 2
        
        temp_origin = frame.point
        
        # ============ BOTTOM LAYER ============
        # Stick 1: along frame's x-axis, starting at origin
        s1_start = temp_origin
        s1 = self._create_stick_from_start(s1_start, frame.xaxis, frame.zaxis, stick_length)
        sticks.append(s1)
        
        # Stick 2: along frame's y-axis
        # Original: temp_origin + Vector(depth/2, depth/2, 0)
        s2_start = temp_origin + frame.xaxis * half_depth + frame.yaxis * half_depth
        s2 = self._create_stick_from_start(s2_start, frame.yaxis, frame.zaxis, stick_length)
        sticks.append(s2)
        
        # Stick 3: along frame's x-axis
        # Original: stick_2.axis.end + Vector(depth/2, -depth/2, 0)
        s3_start = s2.end_point + frame.xaxis * half_depth + frame.yaxis * (-half_depth)
        s3 = self._create_stick_from_start(s3_start, frame.xaxis, frame.zaxis, stick_length)
        sticks.append(s3)
        
        # Stick 4: along -frame's y-axis
        # Original: stick_3.axis.end + Vector(-depth/2, -depth/2, 0)
        s4_start = s3.end_point + frame.xaxis * (-half_depth) + frame.yaxis * (-half_depth)
        s4 = self._create_stick_from_start(s4_start, -frame.yaxis, frame.zaxis, stick_length)
        sticks.append(s4)
        
        # ============ VERTICAL STICKS ============
        # Stick 5: Original: temp_origin + Vector(width/2, 0, width/2)
        s5_start = temp_origin + frame.xaxis * half_width + frame.zaxis * half_width
        s5 = self._create_stick_from_start(s5_start, frame.zaxis, frame.yaxis, stick_vert_length)
        sticks.append(s5)
        
        # Stick 6: Original: stick_2.axis.end + Vector((width/2)-(depth/2), -(depth/2), (width/2))
        s6_start = s2.end_point + frame.xaxis * (half_width - half_depth) + frame.yaxis * (-half_depth) + frame.zaxis * half_width
        s6 = self._create_stick_from_start(s6_start, frame.zaxis, frame.yaxis, stick_vert_length)
        sticks.append(s6)
        
        # Stick 7: Original: stick_3.axis.end + Vector(-width/2, 0, width/2)
        s7_start = s3.end_point + frame.xaxis * (-half_width) + frame.zaxis * half_width
        s7 = self._create_stick_from_start(s7_start, frame.zaxis, frame.yaxis, stick_vert_length)
        sticks.append(s7)
        
        # Stick 8: Original: stick_4.axis.end + Vector((depth/2)-(width/2), depth/2, width/2)
        s8_start = s4.end_point + frame.xaxis * (half_depth - half_width) + frame.yaxis * half_depth + frame.zaxis * half_width
        s8 = self._create_stick_from_start(s8_start, frame.zaxis, frame.yaxis, stick_vert_length)
        sticks.append(s8)
        
        # ============ TOP LAYER ============
        z_offset = frame.zaxis * (stick_vert_length + self.width)
        
        # Stick 9: same as s1, moved up
        s9_start = temp_origin + z_offset
        s9 = self._create_stick_from_start(s9_start, frame.xaxis, frame.zaxis, stick_length)
        sticks.append(s9)
        
        # Stick 10: same as s2, moved up
        s10_start = temp_origin + frame.xaxis * half_depth + frame.yaxis * half_depth + z_offset
        s10 = self._create_stick_from_start(s10_start, frame.yaxis, frame.zaxis, stick_length)
        sticks.append(s10)
        
        # Stick 11: same pattern as s3
        s11_start = s10.end_point + frame.xaxis * half_depth + frame.yaxis * (-half_depth)
        s11 = self._create_stick_from_start(s11_start, frame.xaxis, frame.zaxis, stick_length)
        sticks.append(s11)
        
        # Stick 12: same pattern as s4
        s12_start = s11.end_point + frame.xaxis * (-half_depth) + frame.yaxis * (-half_depth)
        s12 = self._create_stick_from_start(s12_start, -frame.yaxis, frame.zaxis, stick_length)
        sticks.append(s12)
        
        # ============ CENTER THE CUBE ============
        all_points = []
        for stick in sticks:
            box = stick.geometry
            all_points.extend(box.points)
        
        bbox = bounding_box(all_points)
        bbox_center = Point(
            sum(p[0] for p in bbox) / len(bbox),
            sum(p[1] for p in bbox) / len(bbox),
            sum(p[2] for p in bbox) / len(bbox)
        )
        
        offset = frame.point - bbox_center
        
        for stick in sticks:
            new_center = stick.center_frame.point + offset
            stick.center_frame = Frame(new_center, stick.center_frame.xaxis, stick.center_frame.yaxis)
            stick.axis = Line(
                stick.center_frame.point - stick.center_frame.xaxis * (stick.length / 2),
                stick.center_frame.point + stick.center_frame.xaxis * (stick.length / 2)
            )
        
        return sticks
    
