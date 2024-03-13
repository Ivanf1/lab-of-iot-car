import argparse
from pi_face_recognition import FaceRecognitor
from cube_finder import CubeFinder
from line_follower import LineFollower
from mqtt_client import MqttClient
from path_computer import PathComputer
from config import Config
from time import sleep
import json

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-c", "--cascade", required=True, help="path to where the face cascade resides")
ap.add_argument("-e", "--encodings", required=True, help="path to serialized db of facial encodings")
args = vars(ap.parse_args())

config = Config()
path_follower = PathComputer()
lf = LineFollower()
mqtt_client = MqttClient(config=config)
mqtt_client.start()

fc = FaceRecognitor(args["encodings"], args["cascade"])
face_uuid = fc.start()

print(face_uuid)

cube_finder = CubeFinder(config=config)
stops = cube_finder.get_cube_positions_for_person(face_uuid)

print(stops)

stops_temp = []
for s in stops:
    stops_temp.append(s)

mqtt_client.publish_message_async("sm_iot_lab/car/route/start", json.dumps(stops_temp))

actions = path_follower.compute_path(stops=stops)

i = 0
lf.new_path(actions)

while i < len(stops_temp):
    print("waiting for cube release")

    pickup_point_n = stops_temp[i][1]
    cube_dropper_n = stops_temp[i][3]

    mqtt_client.publish_message_sync("sm_iot_lab/pickup_point/"+ pickup_point_n +"/cube/"+ cube_dropper_n +"/release/request")

    print("cube released, continuing")

    mqtt_client.publish_message_async("sm_iot_lab/car/route/update", stops_temp[i])

    i += 1
    lf.continue_path()

mqtt_client.publish_message_async("sm_iot_lab/car/route/end", "")

print("path completed")