#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to record and save data about TALOS' motions """ 
# ---------------------------------------------------------------------------
#                                  IMPORTS
# ---------------------------------------------------------------------------
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
import os
# ---------------------------------------------------------------------------


class CollectDataTalos(object): 

    def __init__(self, action_name):
        
        super(CollectDataTalos,self).__init__()

        # Initiialization - 32 joints of interest
        self._ee_pose_r = Pose()
        self._ee_pose_l = Pose()
        self._joint_pos = [0] * 32
        self._joint_vel = [0] * 32
        self._joint_eff = [0] * 32

        # Ros subscribers
        rospy.Subscriber('/ee_state_r', PoseStamped, self.callback_ee_state_r)
        rospy.Subscriber('/ee_state_l', PoseStamped, self.callback_ee_state_l) 
        rospy.Subscriber('/joint_states', JointState, self.callback_joint_state)

        # Definition of the location and the names of the files to save
        folder_name = os.getcwd() + "/data/talos_" + action_name
        self._filepose_r = open(folder_name+"_pose_r.csv","w+")
        self._filepose_l = open(folder_name+"_pose_l.csv","w+")
        self._filejointpos = open(folder_name+"_jointpos.csv","w+")
        self._filejointvel = open(folder_name+"_jointvel.csv","w+")
        self._filejointeff = open(folder_name+"_jointeff.csv","w+")

        # Same rate as the Shimmer3 GSR+ sensor
        self._rate = 50.33

        # Boolean that indicates if the data collection is over or not
        self.finished = False


    def callback_ee_state_r(self,data):
        self._ee_pose_r = data.pose # right end effector 3D position

    def callback_ee_state_l(self,data):
        self._ee_pose_l = data.pose # left end effector 3D position

    def callback_joint_state(self,data):
        self._joint_pos = data.position # joints position
        self._joint_vel = data.velocity # joints velocity
        self._joint_eff = data.effort # joints effort

    def activate_thread(self):
        # set control rate
        control_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown() and not self.finished:
            self._dump_data()
            control_rate.sleep()

    def _dump_data(self):
        pose_r = [0] * 7
        pose_r[0] = self._ee_pose_r.position.x
        pose_r[1] = self._ee_pose_r.position.y
        pose_r[2] = self._ee_pose_r.position.z
        pose_r[3] = self._ee_pose_r.orientation.x
        pose_r[4] = self._ee_pose_r.orientation.y
        pose_r[5] = self._ee_pose_r.orientation.z
        pose_r[6] = self._ee_pose_r.orientation.w
        # Writing of the data in the specified files
        self.write_data(self._filepose_r, pose_r)

        pose_l = [0] * 7
        pose_l[0] = self._ee_pose_l.position.x
        pose_l[1] = self._ee_pose_l.position.y
        pose_l[2] = self._ee_pose_l.position.z
        pose_l[3] = self._ee_pose_l.orientation.x
        pose_l[4] = self._ee_pose_l.orientation.y
        pose_l[5] = self._ee_pose_l.orientation.z
        pose_l[6] = self._ee_pose_l.orientation.w
        # Writing of the data in the specified files
        self.write_data(self._filepose_l, pose_l)

        self.write_data(self._filejointpos, self._joint_pos)
        self.write_data(self._filejointvel, self._joint_vel)
        self.write_data(self._filejointeff, self._joint_eff)

    def write_data(self, fileobj, data):
        for datum in data:
            fileobj.write("%f, " % datum)
        fileobj.write("\n")

    def clean_shutdown(self):
        self.finished = True
        self._filepose_r.close()
        self._filepose_l.close()
        self._filejointpos.close()
        self._filejointvel.close()
        self._filejointeff.close()

def main():
    # Starting node connection to ROS
    print("Initializing node... ")
    rospy.init_node("talosdatadumper")
    dumper = CollectDataTalos()
    rospy.on_shutdown(dumper.clean_shutdown)
    dumper.activate_thread()

if __name__ == "__main__":
    main()
