#!/usr/bin/env python3

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to publish signal to start recording with both cameras """ 
# ---------------------------------------------------------------------------
#                                  IMPORTS
# ---------------------------------------------------------------------------
import paho.mqtt.client as mqtt
import time
import os
import shlex, subprocess
import sys
# ---------------------------------------------------------------------------

class MqttSubscriber(object):

    def __init__(self):
        super(MqttSubscriber, self).__init__()
        self.mqttBroker = sys.argv[1]
        self.client = mqtt.Client("Host"+str(os.getpid()))
        self.still_on = True

    def on_message(self, client, userdata, message):
        print("on_message")
        id_cam0 = sys.argv[2]
        # If signal to start recording received
        if str(message.payload.decode("utf-8"))=='1': 
            # Command to record both cameras simulatneously
            command_line = "ffmpeg -f v4l2 -y -r 30 -s 1280x720 -i " + id_cam0 + " -c:v h264_nvenc /tmp/out_" + id_cam0.split('/')[-1] + ".avi"
            print("starting",command_line)
            args = shlex.split(command_line)
            self.mediapipe = subprocess.Popen(args)
        # If signal to stop recording received
        if str(message.payload.decode("utf-8"))=='0':
            subprocess.Popen.kill(self.mediapipe) # kill subprocess that do the recording
            self.still_on = False

    def connect(self):
        print("in connect")
        self.client.connect(self.mqttBroker)
        print("connected done") 
        self.client.loop_start()
        print("loop started")
        self.client.subscribe("MEDIAPIPE")
        print("subscribe done")
        self.client.on_message=self.on_message
        while self.still_on: 
            time.sleep(1)
        self.client.loop_stop() # subscriber killed when recording over

if __name__ == '__main__':
    mqtt_sub = MqttSubscriber()
    mqtt_sub.connect()
