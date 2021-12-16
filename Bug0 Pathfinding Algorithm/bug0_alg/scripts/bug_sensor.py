import math

# CONSTANTS
READING_SPREAD = 40
FRONT_ANGLE = 0.0
RIGHT_ANGLE = -math.pi / 2

class Sensors:
    def __init__(self):
        self.front = None
        self.right = None
        self.rawData = None

    # angle relative to sensors [-3/4 pi, 3/4 pi]
    def AngleToIndex(self, angle):
        return int((angle - self.rawData.angle_min) / self.rawData.angle_increment)

    def UpdateReadings(self, data):
        self.rawData = data # need to update first so we can access AngleToIndex
        frontIndex = self.AngleToIndex(FRONT_ANGLE)
        rightIndex = self.AngleToIndex(RIGHT_ANGLE)
        self.front = min(data.ranges[(frontIndex - (READING_SPREAD // 2)):(frontIndex + (READING_SPREAD // 2))])
        self.right = min(data.ranges[(rightIndex - (READING_SPREAD // 2)):(rightIndex + (READING_SPREAD // 2))])

    def GetReadings(self):
        return (self.front, self.right)

    # angle in radians, should be in range [-3/4 pi, 3/4 pi]
    def ReadingAtAngle(self, angle):
        try:
            if (angle < (-3 * math.pi / 4)) or (angle > (3 * math.pi / 4)) or (self.rawData == None):
                return -1.0
            leftIndex = self.AngleToIndex(angle) - (READING_SPREAD // 2)
            rightIndex = self.AngleToIndex(angle) + (READING_SPREAD // 2)
            if leftIndex < 0:
                leftIndex = 0
            elif rightIndex >= len(self.rawData.ranges):
                rightIndex = len(self.rawData.ranges) - 1
            return min(self.rawData.ranges[leftIndex:rightIndex])
        except ValueError: # for when leftIndex = rightIndex
            return -1.0
