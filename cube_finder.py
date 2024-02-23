import requests

class CubeFinder:
    baseUrl = "http://localhost:3000/api/person/"

    def get_cube_positions_for_person(self, person_uuid):
        response = requests.get(self.baseUrl + person_uuid + "/cubes")
        response_json = response.json()

        stops = self.build_stops_list(json=response_json)
        return stops
    
    def build_stops_list(self, json):
        stops = []

        for cube in json:
            pickup_point_position = str(cube["cube_dropper"]["pickup_point"]["position"]) 
            cube_dropper_position = str(cube["cube_dropper"]["position"])
            current_stop = "P" + pickup_point_position + "C" + cube_dropper_position
            stops.append(current_stop)

        stops.sort()
        return stops
