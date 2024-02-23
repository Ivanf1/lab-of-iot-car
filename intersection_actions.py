from enum import Enum

class IntersectionAction(Enum):
    LEFT = 0
    RIGHT = 1
    IGNORE = 2
    STOP = 3
    ROTATE_TO_OPPOSITE_DIRECTION = 4