#!/usr/bin/env python3
# ---------------------------------------------------------------------------
import paho.mqtt.client as mqtt
import sys
mqttBroker = sys.argv[1]
client = mqtt.Client("trigger")
client.connect(mqttBroker)
print("connected done") 
client.loop_start()
print("loop started")
client.publish("MEDIAPIPE","1")
import time
time.sleep(4)
client.publish("MEDIAPIPE","0")


client.loop_stop() # subscriber killed when recording over

