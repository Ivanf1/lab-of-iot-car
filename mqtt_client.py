import paho.mqtt.client as mqtt

class MqttClient:
    broker_address = "localhost"
    mqttc = None
    connected = False

    waiting = False
    got_response = False

    def __init__(self) -> None:
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_message = self.on_message

        self.mqttc.connect(self.broker_address, 1883, 60)

    # The callback for when the client receives a CONNACK response from the server.
    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected with result code {reason_code}")
        # Subscribing in on_connect() means that if we lose the connection and
        # reconnect then subscriptions will be renewed.
        client.subscribe("sm_iot_lab/pickup_point/+/cube/+/release/response")
        self.connected = True

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))
        if msg.topic.endswith("release/response"):
            self.got_response = True

    def start(self):
        self.mqttc.loop_start()

        while not self.connected:
            continue

        self.mqttc.loop_stop()

    def publish_message_sync(self, topic):
        self.waiting = True
        self.mqttc.loop_start()

        self.mqttc.publish(topic=topic)

        while not self.got_response:
            continue

        self.mqttc.loop_stop()
        self.got_response = False
        self.waiting = False


# mqtt_client = MqttClient()
# mqtt_client.start()
# mqtt_client.publish_message_sync("sm_iot_lab/pickup_point/0/cube/0/release/request")