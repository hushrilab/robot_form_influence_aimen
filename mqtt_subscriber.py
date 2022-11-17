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
import time
import os
import shlex, subprocess
# ---------------------------------------------------------------------------

class MqttSubscriber(object):

    def __init__(self):
        super(MqttSubscriber, self).__init__()
        self.mqttBroker = "localhost"
        self.client = mqtt.Client("Host")
        self.still_on = True

    def on_message(self, client, userdata, message):
	print("on_message")
        # Folder to save the videos
        path = '/home/aimen/robohub/julie/talos_public_ws/data/videos/' 
        # Index of the cameras /dev/videoX
        id_cam0 = "/dev/video2" #"/dev/v4l/by-id/usb-SunplusIT_Inc_Streaming_Webcam_JML20201021V1-video-index0"
        id_cam1 = "/dev/video4" #"/dev/v4l/by-id/usb-ANYKA_HD_WebCam_2MP_12345-video-index0"
        # If signal to start recording received
        if str(message.payload.decode("utf-8"))=='1': 
            # Command to record both cameras simulatneously
            command_line = "ffmpeg -f video4linux2 -r 30 -s 1280x720 -i " + id_cam0 + " -r 30 -s 1280x720 -i " + id_cam1 + " -map 0 " + str(path) + "out0.avi -map 1 " + str(path) + "out1.avi"
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
