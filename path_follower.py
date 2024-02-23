import path_finder

class PathFollower:
    path_finder = None

    def __init__(self) -> None:
        self.path_finder = path_finder.PathFinder()

    def follow_path(self, stops):
        path, directions = self.path_finder.compute_path(stops=stops)

        for idx, path_step in enumerate(path):
            if idx > len(directions) - 1:
                print("Destination reached")
                return
                
            if path_step in stops:
                print("Stopping at: " + path_step)
                stops.remove(path_step)
                print("Remaining stops: " + str(len(stops)))

            print("Current: " + path_step + " | next direction: " + directions[idx])

path_follower = PathFollower()
path_follower.follow_path(["P0C0", "P1C1"])

