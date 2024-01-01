import line_follower
from line_follower import IntersectionAction

# actions = [IntersectionAction.RIGHT, IntersectionAction.LEFT, IntersectionAction.IGNORE, IntersectionAction.LEFT, IntersectionAction.LEFT, IntersectionAction.LEFT, IntersectionAction.RIGHT, IntersectionAction.STOP]
actions = [IntersectionAction.RIGHT, IntersectionAction.LEFT, IntersectionAction.ROTATE_TO_OPPOSITE_DIRECTION, IntersectionAction.RIGHT, IntersectionAction.LEFT, IntersectionAction.STOP]

lf = line_follower.LineFollower()
lf.do_follow_line(actions)