# bridge.py
# Minimal bridging stub to keep pipeline intact.
# You can later replace this with a real non-coplanar bridging algorithm.

from stick_fixed import Stick


class BridgingModule(object):
    def __init__(self, stick_list, stick_length, width, depth):
        self.sticks = stick_list or []
        self.stick_length = float(stick_length)
        self.width = float(width)
        self.depth = float(depth)

    def build(self):
        """Currently no automatic bridging. Returns empty list."""
        return []
