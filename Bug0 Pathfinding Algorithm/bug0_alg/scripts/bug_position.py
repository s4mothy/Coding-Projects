import math
import sys

# CONSTANTS
THETA_OFFSET = 0.05

class Position:
    def __init__(self):
        self.cx = None
        self.cy = None
        self.ct = None

    def UpdatePosition(self, x, y, t):
        self.cx, self.cy, self.ct = (x, y, t)

    def GetPosition(self):
        return (self.cx, self.cy, self.ct)

    def DistToGoal(self, tx, ty):
        if None in (self.cx, self.cy):
            return sys.maxsize
        return math.sqrt((tx - self.cx)**2 + (ty - self.cy)**2)

    # theta of goal relative in scanner angular coordinates
    def LocalGoalTheta(self, tx, ty):
        if None in (self.cx, self.cy, self.ct):
            return math.nan
        lgt = math.atan2(ty - self.cy, tx - self.cx) - self.ct
        if lgt < -math.pi:
            lgt += 2 * math.pi
        return lgt
    
    def FacingGoal(self, tx, ty):
        if None in (self.cx, self.cy, self.ct):
            return False
        lgt = self.LocalGoalTheta(tx, ty)
        return -THETA_OFFSET <= lgt <= THETA_OFFSET

    def AtTargetTheta(self, tt):
        return -THETA_OFFSET <= (self.ct - tt) <= THETA_OFFSET

    



