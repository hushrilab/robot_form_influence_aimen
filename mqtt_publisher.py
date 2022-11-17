#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to publish signal to start recording with both cameras """ 
# ---------------------------------------------------------------------------
#                                  IMPORTS
# ---------------------------------------------------------------------------
import paho.mqtt.client as mqtt 
from random import randrange, uniform
import time
# ---------------------------------------------------------------------------


class MqttPublisher(object):

    def __init__(self):
        super(MqttPublisher, self).__init__()
        self.mqttBroker = "localhost"
        self.client = mqtt.Client("MediaPipeFlag")

    def publish(self,nb):
        self.client.connect(self.mqttBroker) # connection to the broker
        self.client.publish("MEDIAPIPE", nb) # publish to the topic MEDIAPIPE


if __name__ == '__main__':
    mqtt_pub = MqttPublisher()
    mqtt_pub.publish(1)
    time.sleep(3)
    mqtt_pub.publish(0)
