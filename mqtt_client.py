import paho.mqtt.client as mqtt

class MqttClient:
    config = None
    # broker_address = "192.168.1.82"
    broker_address = "192.168.1.21"
    mqttc = None
    connected = False

    waiting = False
    got_response = False

    got_server_ip = False

    def __init__(self, config) -> None:
        self.config = config

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
        client.subscribe("sm_iot_lab/server/ip")
        self.connected = True

    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        print(msg.topic+" "+str(msg.payload))
        if msg.topic.endswith("release/response"):
            self.got_response = True

        if msg.topic.endswith("server/ip"):
            ip = str(msg.payload)[2:len(str(msg.payload))-1]
            self.config.set_server_ip(ip)
            self.got_server_ip = True

    def start(self):
        self.mqttc.loop_start()

        while not self.connected:
            continue
        while not self.got_server_ip:
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

    def publish_message_async(self, topic, message):
        self.mqttc.loop_start()

        self.mqttc.publish(topic=topic, payload=message)

        self.mqttc.loop_stop()