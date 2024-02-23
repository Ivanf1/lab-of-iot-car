from intersection_actions import IntersectionAction
import path_finder

class PathComputer:
    path_finder = None

    def __init__(self) -> None:
        self.path_finder = path_finder.PathFinder()

    def compute_path(self, stops):
        path, directions = self.path_finder.compute_path(stops=stops)

        return self.directions_to_intersection_actions(path=path, stops=stops, directions=directions)

    def directions_to_intersection_actions(self, path, stops, directions):
        actions = []

        for idx, path_step in enumerate(path):
            if idx > len(directions) - 1:
                continue
            if path_step in stops:
                actions.append("stop")
                if path_step == "P0C0" and path[idx + 1] == "P0_intersection":
                    actions.append("turn_around")
                if path_step == "P1C0" and path[idx + 1] == "P1_intersection":
                    actions.append("turn_around")
                if path_step == "P0C1" and path[idx + 1] == "P0C0":
                    actions.append("turn_around")
                if path_step == "P1C1" and path[idx + 1] == "P1C0":
                    actions.append("turn_around")
                stops.remove(path_step)
            actions.append(directions[idx])

        actions_enums = []
        for action in actions:
            a = None
            if action == "straight":
                a = IntersectionAction.IGNORE
            if action == "l":
                a = IntersectionAction.LEFT
            if action == "r":
                a = IntersectionAction.RIGHT
            if action == "stop":
                a = IntersectionAction.STOP
            if action == "turn_around":
                a = IntersectionAction.ROTATE_TO_OPPOSITE_DIRECTION
            actions_enums.append(a)
        
        return actions_enums