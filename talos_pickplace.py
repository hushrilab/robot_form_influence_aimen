#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to contain all functions for pickplace task for TIAGo """ 
# ---------------------------------------------------------------------------
#                                  IMPORTS
# ---------------------------------------------------------------------------
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import numpy as np
from std_msgs.msg import String, Duration
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Transform
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix,quaternion_from_matrix
import pick_and_place_constants_without_user as const
#import pick_and_place_constants_without_user_45 as const
from math import ceil
from talos_gl import TalosGl
from publisher_ee_state import PublisherEeState
from collect_data_talos import CollectDataTalos
#from collect_data_shimmer import CollectDataShimmer
import threading
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import os
import shlex, subprocess
# ---------------------------------------------------------------------------
mqtt_enable = False
shimmer_enable = False

if mqtt_enable:
	from mqtt_publisher import MqttPublisher
if shimmer_enable:
	from collect_data_shimmer import CollectDataShimmer

class TalosPickPlace(TalosGl):
	"""TalosPickPlace"""

	# ============================ INITIALIZATION ============================
	def __init__(self):
		TalosGl.__init__(self, const)

		self.move_groups = []
		self.move_l_groups = []
		self.gripper_groups = []
		self.touch_links_groups = []
		self.eef_links = []

		for i in range(const.NB_GROUPS):

			group_name = const.GROUP_NAME[i]
			l_group_name = const.LARGE_GROUP_NAME[i]
			gripper_name = const.GRIPPER_NAME[i]
			move_group = moveit_commander.MoveGroupCommander(group_name)
			move_group.set_planning_time(const.PLANNING_TIME)
			move_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
			move_l_group = moveit_commander.MoveGroupCommander(l_group_name)
			move_l_group.set_planning_time(const.PLANNING_TIME)
			move_l_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
			gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
			gripper_group.set_end_effector_link(const.EEF[i])
			touch_links = self.robot.get_link_names(group=gripper_name)
			for item in self.robot.get_link_names(group=const.GRIPPER_GROUP[i]):
				if item not in touch_links:
					touch_links.append(item)
			eef_link = move_group.get_end_effector_link()
			self.move_groups.append(move_group)
			self.move_l_groups.append(move_l_group)
			self.gripper_groups.append(gripper_group)
			self.touch_links_groups.append(touch_links)
			self.eef_links.append(eef_link)

		self.joints_right_relax = []
		self.joints_left_relax = []

		self.joints_right_relax = [0.0, -0.15, 0.4, -0.7, 0.0, 0.0, 0.0]
		self.joints_left_relax = [ 0.0, 0.15, -0.4, -0.7, 0.0, 0.0, 0.0]


		self.const = const
			
		self.pub_ee_state = PublisherEeState(self.move_groups[0], self.move_groups[1])
		self.talos_data_collecter = CollectDataTalos("pickplace")
		if shimmer_enable:
			self.shimmer_data_collecter = CollectDataShimmer()

		# Create one thread for publisher
		t1 = threading.Thread(name="ee", target=self.pub_ee_state.ee_state_talker, args=())
		t1.start()

		# Initialization of the mqtt publisher for the recording of the cameras
		if mqtt_enable:
			self.mqtt_pub = MqttPublisher()

		rospy.on_shutdown(self.clean_shutdown)

		print "Initialization done for Talos PICK & PLACE task."
		print ""

	# ========================================================================


	# ============================= PICK PIPELINE ============================
	def pick(self, obj_to_grasp, id_group): 

		move_group = self.move_groups[id_group]
		move_l_group = self.move_l_groups[id_group]
		eef_link = self.eef_links[id_group]
		touch_links = self.touch_links_groups[id_group]
		self.pose_obj = self.scene.get_object_poses([obj_to_grasp]).get(obj_to_grasp)

		# DETERMINE GRIPPER ORIENTATION
		quat_obj = self.pose_obj.orientation #base
		rot_obj = quaternion_matrix([quat_obj.x,quat_obj.y,quat_obj.z,quat_obj.w]) #base
		orientation = np.dot(rot_obj,np.dot(self.R_base_wrist_r,self.R_constraints_wrist_r)) # wrist
		orientation = np.dot(np.linalg.inv(self.R_base_wrist_r),orientation) #base
		quat_goal = quaternion_from_matrix(orientation)

		# MOVE TO APPROACH GRASP POSE
		# Get object pose in base link frame + wrist orientation
		obj_pose_base = PoseStamped()
		obj_pose_base.header.frame_id = self.robot.get_planning_frame()
		obj_pose_base.pose.position.x = self.pose_obj.position.x
		obj_pose_base.pose.position.y = self.pose_obj.position.y
		obj_pose_base.pose.position.z = self.pose_obj.position.z
		obj_pose_base.pose.orientation = Quaternion(quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])
		dim_z = self.get_dim_obj(obj_to_grasp,2)
		if id_group == 0: #right grabs from bottom object
			obj_pose_base.pose.position.z = obj_pose_base.pose.position.z - dim_z/4
		elif id_group == 1: #left grabs from up
			obj_pose_base.pose.position.z = obj_pose_base.pose.position.z + dim_z/4
		# change ref frame from base to wrist
		obj_pose_wrist = np.dot(self.R_base_wrist_r,np.array([obj_pose_base.pose.position.x,obj_pose_base.pose.position.y,obj_pose_base.pose.position.z,1]))
		obj_pose_wrist = np.dot(obj_pose_wrist,rot_obj)
		#add in approach in wrist frame
		dim_y = self.get_dim_obj(obj_to_grasp,1)
		obj_pose_wrist[0] = obj_pose_wrist[0] - self.const.APPROACH_PICK_X[id_group] - self.const.MARGIN_X[id_group]
		obj_pose_wrist[1] = obj_pose_wrist[1] - self.const.APPROACH_PICK_Y[id_group] - self.const.MARGIN_Y[id_group] - self.const.DIST2 - dim_y/2
		obj_pose_wrist[2] = obj_pose_wrist[2] +  self.const.APPROACH_PICK_Z[id_group] + self.const.MARGIN_Z[id_group]
		obj_pose_wrist = np.dot(rot_obj,obj_pose_wrist)
		# change ref frame from wrist to base
		obj_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), obj_pose_wrist)
		# define goal pose in base frame
		obj_pose_base.pose.position.x = obj_pose_base_v[0]
		obj_pose_base.pose.position.y = obj_pose_base_v[1]
		obj_pose_base.pose.position.z = obj_pose_base_v[2]

		success = self.move_attempt(id_group, move_group, obj_pose_base, "grasp_approach_pose")
		#if success == False:
			#success = self.move_attempt(id_group, move_l_group, obj_pose_base, "grasp_approach_pose", True)


		# OPEN GRIPPER
		if self.steps==True:
			print "Open gripper ?"
			raw_input()
		self.open_gripper(id_group)

		if self.steps==True:
			print "Grasp pose ?"
			raw_input()

		# MOVE TO GRASP POSE
		grasp_approach_pose= obj_pose_base
		grasp_pose_base = grasp_approach_pose
		# change ref frame from base to wrist
		grasp_pose_wrist = np.dot(self.R_base_wrist_r,np.array([grasp_pose_base.pose.position.x,grasp_pose_base.pose.position.y,grasp_pose_base.pose.position.z,1]))
		grasp_pose_wrist = np.dot(grasp_pose_wrist,rot_obj)
		grasp_pose_wrist[0] = grasp_pose_wrist[0] + self.const.APPROACH_PICK_X[id_group]
		grasp_pose_wrist[1] = grasp_pose_wrist[1] + self.const.APPROACH_PICK_Y[id_group]
		grasp_pose_wrist[2] = grasp_pose_wrist[2] -  self.const.APPROACH_PICK_Z[id_group]
		grasp_pose_wrist = np.dot(rot_obj,grasp_pose_wrist)
		# change ref frame from wrist to base
		grasp_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), grasp_pose_wrist)
		# define goal pose in base frame
		grasp_pose_base.pose.position.x = grasp_pose_base_v[0]
		grasp_pose_base.pose.position.y = grasp_pose_base_v[1]
		grasp_pose_base.pose.position.z = grasp_pose_base_v[2]
		
		# Go to grasp pose through cartesian path
		waypoints = []
		waypoints.append(grasp_pose_base.pose)
		# WITH TORSO LEANING
		(plan, fraction) = move_l_group.compute_cartesian_path(waypoints,0.01,0.0)
		# WITHOUT TORSO LEANING
		#(plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
		if len(plan.joint_trajectory.points)!=0:
			print "Executing plan found to grasp_pose"
			# WITH TORSO LEANING
			move_l_group.execute(plan, wait=True)
			# WITHOUT TORSO LEANING
			#move_group.execute(plan, wait=True)
		else:
			print "[ERROR] No plan found to reach grasp_pose"
			sys.exit(1)


		# CLOSE GRIPPER
		if self.steps==True:
			print "Close gripper ?"
			raw_input()
		self.close_gripper(id_group)

		# ATTACH OBJECT
		self.scene.attach_box(eef_link, obj_to_grasp, touch_links=touch_links)

		# MOVE TO RETREAT POSE
		#if self.steps==True:
		#	print "Retreat pose ?"
		#	raw_input()
		pick_retreat_pose_base = grasp_pose_base
		# change ref frame from base to wrist
		pick_retreat_pose_wrist = np.dot(self.R_base_wrist_r,np.array([pick_retreat_pose_base.pose.position.x,pick_retreat_pose_base.pose.position.y,pick_retreat_pose_base.pose.position.z,1]))
		pick_retreat_pose_wrist = np.dot(pick_retreat_pose_wrist,rot_obj)
		pick_retreat_pose_wrist[0] = pick_retreat_pose_wrist[0] - self.const.RETREAT_X[id_group]
		pick_retreat_pose_wrist[1] = pick_retreat_pose_wrist[1] - self.const.RETREAT_Y[id_group]
		pick_retreat_pose_wrist[2] = pick_retreat_pose_wrist[2] +  self.const.RETREAT_Z[id_group]
		pick_retreat_pose_wrist = np.dot(rot_obj,pick_retreat_pose_wrist)
		# change ref frame from wrist to base
		pick_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), pick_retreat_pose_wrist)
		# define goal pose in base frame
		pick_retreat_pose_base.pose.position.x = pick_retreat_pose_base_v[0]
		pick_retreat_pose_base.pose.position.y = pick_retreat_pose_base_v[1]
		pick_retreat_pose_base.pose.position.z = pick_retreat_pose_base_v[2]
		#success = self.move_attempt(id_group, move_l_group, pick_retreat_pose_base, "pick_retreat_pose")
		#success = self.move_attempt(id_group, move_group, pick_retreat_pose_base, "pick_retreat_pose")

		print "Pick pipeline done."
		print ""

		return pick_retreat_pose_base
	# ========================================================================


	# ============================= PLACE PIPELINE ===========================
	def place(self, obj_to_grasp, last_pose, id_group):

		move_group = self.move_groups[id_group]
		move_l_group = self.move_l_groups[id_group]
		eef_link = self.eef_links[id_group]
		touch_links = self.touch_links_groups[id_group]

		# DETERMINE GRIPPER ORIENTATION
		quat_obj = self.pose_obj.orientation #base
		rot_obj = quaternion_matrix([quat_obj.x,quat_obj.y,quat_obj.z,quat_obj.w]) #base
		rot_obj = np.linalg.inv(rot_obj)
		orientation = np.dot(rot_obj,np.dot(self.R_base_wrist_l,self.R_constraints_wrist_l)) # wrist
		orientation = np.dot(np.linalg.inv(self.R_base_wrist_l),orientation) #base
		quat_goal = quaternion_from_matrix(orientation)
		
		print "Left hand starts putting object down"

		# MOVE TO APPROACH PLACE POSE
		dim_y = self.get_dim_obj(obj_to_grasp,1)
		place_approach_pose_base = self.pose_obj
		place_approach_pose_base.position.x = self.pose_obj.position.x + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[0]
		place_approach_pose_base.position.y = self.pose_obj.position.y + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[1]
		place_approach_pose_base.position.z = self.pose_obj.position.z + self.const.OBJ_POS_GRASP.get(obj_to_grasp).get("translation")[2]
		place_approach_pose_base.orientation = Quaternion(quat_goal[0],quat_goal[1],quat_goal[2],quat_goal[3])

		# change ref frame from base to wrist
		place_approach_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_approach_pose_wrist = np.dot(place_approach_pose_wrist,rot_obj)

		#add in approach in wrist frame
		place_approach_pose_wrist[0] = place_approach_pose_wrist[0] + self.const.APPROACH_PLACE_X[id_group] + self.const.MARGIN_X[id_group]
		place_approach_pose_wrist[1] = place_approach_pose_wrist[1] + self.const.APPROACH_PLACE_Y[id_group] - self.const.DIST1 -  dim_y/2 + self.const.MARGIN_Y[id_group]
		place_approach_pose_wrist[2] = place_approach_pose_wrist[2] + self.const.APPROACH_PLACE_Z[id_group] + (ceil(self.const.APPROACH_PICK_Z[id_group]/2 * 10 ** 2) / 10 ** 2) + self.const.MARGIN_Z[id_group]
		place_approach_pose_wrist = np.dot(rot_obj,place_approach_pose_wrist)

		# change ref frame from wrist to base
		place_approach_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_approach_pose_wrist)
		# define goal pose in base frame
		place_approach_pose_base.position.x = place_approach_pose_base_v[0]
		place_approach_pose_base.position.y = place_approach_pose_base_v[1]
		place_approach_pose_base.position.z = place_approach_pose_base_v[2]
		# WITH TORSO LEANING
		success = self.move_attempt(id_group, move_l_group, place_approach_pose_base, "place_approach_pose")
		# WITHOUT TORSO LEANING
		#success = self.move_attempt(id_group, move_group, place_approach_pose_base, "place_approach_pose")

		# MOVE TO PLACE POSE
		place_pose_base = place_approach_pose_base
		# change ref frame from base to wrist
		place_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_approach_pose_base.position.x,place_approach_pose_base.position.y,place_approach_pose_base.position.z,1]))
		place_pose_wrist = np.dot(place_pose_wrist,rot_obj)
		place_pose_wrist[0] = place_pose_wrist[0] - self.const.APPROACH_PLACE_X[id_group]
		place_pose_wrist[1] = place_pose_wrist[1] - self.const.APPROACH_PLACE_Y[id_group]
		place_pose_wrist[2] = place_pose_wrist[2] - self.const.APPROACH_PLACE_Z[id_group]
		place_pose_wrist = np.dot(rot_obj,place_pose_wrist)
		# change ref frame from wrist to base
		place_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_pose_wrist)
		# define goal pose in base frame
		place_pose_base.position.x = place_pose_base_v[0]
		place_pose_base.position.y = place_pose_base_v[1]
		place_pose_base.position.z = place_pose_base_v[2]

		if self.steps==True:
			print "Place position ?"
			raw_input()

		# Go to grasp pose through cartesian path
		waypoints = []
		waypoints.append(place_pose_base)
		# WITH TORSO LEANING
		(plan, fraction) = move_l_group.compute_cartesian_path(waypoints,0.01,0.0)
		# WITHOUT TORSO LEANING
		#(plan, fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)
		if len(plan.joint_trajectory.points)!=0:
			print "Executing plan found to grasp_pose"
			# WITH TORSO LEANING
			move_l_group.execute(plan, wait=True)
			# WITHOUT TORSO LEANING
			#move_group.execute(plan, wait=True)
		else:
			print "[ERROR] No plan found to reach grasp_pose"
			sys.exit(1)
		#success = self.move_attempt(id_group, move_group, place_pose_base, "place_pose")
		#if success == False:
		#	success = self.move_attempt(id_group, move_l_group, place_pose_base, "place_pose", True)

		# OPEN GRIPPER
		if self.steps==True:
			print "Open left gripper ?"
			raw_input()
		self.open_gripper(id_group)

		# DETACH OBJECT
		self.scene.remove_attached_object(eef_link, name=obj_to_grasp)

		# RETURN TO RETREAT POSE
		place_retreat_pose_base = place_pose_base
		# change ref frame from base to wrist
		place_retreat_pose_wrist = np.dot(self.R_base_wrist_l,np.array([place_retreat_pose_base.position.x,place_retreat_pose_base.position.y,place_retreat_pose_base.position.z,1]))
		place_retreat_pose_wrist = np.dot(place_retreat_pose_wrist, rot_obj)
		place_retreat_pose_wrist[0] = place_retreat_pose_wrist[0] - self.const.RETREAT_PL_X[id_group]
		place_retreat_pose_wrist[1] = place_retreat_pose_wrist[1] - self.const.RETREAT_PL_Y[id_group]
		place_retreat_pose_wrist[2] = place_retreat_pose_wrist[2] - self.const.RETREAT_PL_Z[id_group]
		place_retreat_pose_wrist = np.dot(rot_obj,place_retreat_pose_wrist)
		# change ref frame from wrist to base
		place_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_l), place_retreat_pose_wrist)
		# define goal pose in base frame
		place_retreat_pose_base.position.x = place_retreat_pose_base_v[0]
		place_retreat_pose_base.position.y = place_retreat_pose_base_v[1]
		place_retreat_pose_base.position.z = place_retreat_pose_base_v[2]

		#if self.steps==True:
		#	print "Retreat position ?"
		#	raw_input()

		#success = self.move_attempt(id_group, move_group, place_retreat_pose_base, "place_retreat_pose")
		#if success == False:
		#	success = self.move_attempt(id_group, move_l_group, place_retreat_pose_base, "place_retreat_pose", True)

		print "Place pipeline done."
		print ""
	# ========================================================================


	# ============================ ATTEMPT TO MOVE ===========================
	def move_attempt(self, id_group, group, pose, name, second_attempt=False):
		if second_attempt :
			print "2nd attempt..."
		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			if nb_try < 14:
				is_ok = self.check_plan(id_group,group,plan,name)
			else:
				is_ok = self.check_plan(id_group,group,plan,name,almost_done=True)
			nb_try = nb_try + 1
		if is_ok:
			print "Executing plan found to move to %s." % name
			group.execute(plan)
			group.stop()
			group.clear_pose_targets()
			print "Position %s reached." % name
			print ""
			return True
		else:
			print "[ERROR] No plan found to move to %s." % name
			print ""
			sys.exit(1)
	# ========================================================================


	# =========================== CHECK PLAN IS OK ===========================
	def check_plan(self, id_group, group, plan, name, second_attempt=False):
		""" checks if plan found and if arm_l_1 arm_l_2 arm_r_1 arm_r_2 joint values"""
		nb_pts = len(plan.joint_trajectory.points)
		if nb_pts == 0 : # No plan found
			if second_attempt :
				print "[ERROR] No plan found to move to %s." % name
				print ""
				sys.exit(1)
			return False

		nb_joints = len(group.get_active_joints()) # if 7 => simple group, if 9 => torso included
		if nb_joints == 9:
			ind = 2
		else:
			ind = 0
		print "Checking plan..."
		for i in range(nb_pts):
			print "Point %s:" % i
			pt = plan.joint_trajectory.points[i]
			joints = pt.positions
			j1 = joints[ind] #joint arm_l/r_1
			j2 = joints[ind + 1] #joint arm_l/r_2
			print "J1 : %s" % j1
			print "J2 : %s" % j2
			if id_group == 0: #right arm
				if j1 < -0.65:
					print "Plan pb for 1st joint r"
					return False
				if j2 < -1.38: #1.31
					print "Plan pb for 2nd joint r"
					return False
			elif id_group == 1: #left arm
				if j1 > 0.65:
					print "Plan pb for 1st joint l"
					return False
				if j2 > 2.5:
					print "Plan pb for 2nd joint l"
					return False
		print "Plan checked!"
		return True

	# ========================================================================


	# ======================== GET POSE JOINTS' VALUES =======================
	def get_joints_from_pose(self, id_group, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 20:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(id_group,group,plan,name)
			nb_try = nb_try + 1

		if is_ok:
			nb_pts = len(plan.joint_trajectory.points)
			last_pt = plan.joint_trajectory.points[nb_pts-1]
			joints = last_pt.positions
			return joints
		else:
			print "[ERROR] No plan found to move to %s." % name
			print ""
			sys.exit(1)

	# ========================================================================

	# ============================= MOVE TORSO ===============================
	def move_torso(self, id_group, joint_values):

		current_joints = self.move_group_torso_head.get_current_joint_values()
                print(current_joints)   # added this
		target_joints = current_joints
		target_joints[0] = joint_values[0]
		if len(joint_values) == 2 :
			target_joints[1] = joint_values[1]
		if id_group == 1:
			target_joints[2:] = [0.2,0.2]
		else:
			target_joints[2:] = [0.0,0.0]

		self.execute_joints_cmd(self.move_group_torso_head,target_joints,"turn_torso_turn_head")
	# ========================================================================


	# ========================= MOVE TO SWITCH POINT =========================
	def move_switch_point(self, obj_to_grasp):

		# Get the torso to initial position + head init
		self.move_torso(0,[0.0,0.0])

		dim_y = self.get_dim_obj(obj_to_grasp,1)

		# FIRST GROUP MOVES TO SWITCH POINT
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = self.const.SWITCH_POSITION.get("x") - self.const.MARGIN_X[0]
		p.pose.position.y = self.const.SWITCH_POSITION.get("y") - self.const.DIST2 - self.const.MARGIN_Y[0] - dim_y/2
		p.pose.position.z = self.const.SWITCH_POSITION.get("z") - self.const.MARGIN_Z[0]
		quaternion = quaternion_from_euler(self.const.EULER_SWITCH[0][0],self.const.EULER_SWITCH[0][1],self.const.EULER_SWITCH[0][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		joints_right = self.get_joints_from_pose(0,self.move_groups[0],p,"move_right_arm_to_switch_point")

		print "Left arm moves to grab object"

		# SECOND GROUP MOVES TO SWITCH POINT AND GRABS
		self.open_gripper(1)
		p.pose.position.x = p.pose.position.x + self.const.OFFSET_X
		p.pose.position.y = p.pose.position.y + 2 * (self.const.DIST3 + self.const.MARGIN_Y[1]) + dim_y + self.const.APPROACH_PICK_Y[1]
		p.pose.position.z = p.pose.position.z + self.const.APPROACH_PICK_Z[1]
		quaternion = quaternion_from_euler(self.const.EULER_SWITCH[1][0],self.const.EULER_SWITCH[1][1],self.const.EULER_SWITCH[1][2])
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
		joints_left = self.get_joints_from_pose(1,self.move_groups[1],p,"approach_left_arm_near_switch_point")

		joint_cmd = self.move_group_gl_head.get_current_joint_values()
		joint_cmd[2:9] = joints_left
		joint_cmd[9:16] = joints_right
		joint_cmd[16] = 0.2
		joint_cmd[17] = 0.0

		if self.steps==True:
			print "Switch position ?"
			raw_input()

		self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "move_to_switch_turn_head")

		p.pose.position.y = p.pose.position.y - self.const.APPROACH_PICK_Y[1] - self.const.MARGIN_Y[1]/2

		if self.steps==True:
			print "Approach left arm ?"
			raw_input()

		success = self.move_attempt(1,self.move_groups[1], p, "switch_point")
		if success == False:
			print "[ERROR] switch_point - left arm"
			sys.exit(1)

		if self.steps==True:
			print "Close left gripper ?"
			raw_input()

		self.close_gripper(1)
		self.scene.remove_attached_object(self.move_groups[0].get_end_effector_link(), name=obj_to_grasp)
		self.scene.remove_attached_object(self.eef_links[0], name=obj_to_grasp)
		self.scene.attach_box(self.eef_links[1], obj_to_grasp, touch_links=self.touch_links_groups[1])

		# FIRST GROUP UNGRASPS
		if self.steps==True:
			print "Open right gripper ?"
			raw_input()

		self.open_gripper(0)

		#MOVING AWAY
		p.pose.position.y = p.pose.position.y + self.const.RETREAT_Y[1] + self.const.MARGIN_Y[1]/2
		joints_left = self.get_joints_from_pose(1,self.move_groups[1],p,"move_away_left_arm")

		joint_cmd = self.move_group_gl.get_current_joint_values()
		joint_cmd[2:9]=joints_left[:]
		joint_cmd[9:] = self.joints_right_relax
		if self.steps==True:
			print "Move away from switch position ?"
			raw_input()
		self.execute_joints_cmd(self.move_group_gl, joint_cmd, "relax_right_move_away_left")

		if self.steps==True:
			print "Face other table ?"
			raw_input()

		# Get the torso to face other table + head
		self.move_torso(1,[self.const.TORSO_TURN])

		return p.pose
	# ========================================================================


	# ============================= PICK AND PLACE ===========================
	def pick_and_place_all(self):

		self.load_scene()
		rospy.sleep(2)

		if shimmer_enable:
			self.shimmer_data_collecter._dump_data()

		# Create one thread for data collecter
		self.t2 = threading.Thread(name="talos_data", target=self.talos_data_collecter.activate_thread, args=())
		self.t2.start()

		# Publish signal to start recording with both cameras simultaneously
		if mqtt_enable:
			self.mqtt_pub.publish(1) 

		for obj_to_grasp in self.obj_to_grasp:

			if self.steps==True:
				print "Face first table ?"
				raw_input()

			#TURN TORSO - HEAD + RELAX ARM
			joint_cmd = self.move_group_gl_head.get_current_joint_values()
			joint_cmd[0] = -self.const.TORSO_TURN
			joint_cmd[2:9] = self.joints_left_relax
			joint_cmd[9:16] = self.joints_right_relax
			joint_cmd[16] = 0.2
			joint_cmd[17] = -0.2
			self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "turn_torso_relax_arms")

			if self.steps==True:
				print "Launch pick action ?"
				raw_input()

		  	last_pose = self.pick(obj_to_grasp, 0)

		  	if self.steps==True:
		  		print "Launch switch point action ?"
				raw_input()

			last_pose = self.move_switch_point(obj_to_grasp)

			if self.steps==True:
				print "Launch place action ?"
				raw_input()

			self.place(obj_to_grasp, last_pose, 1)

			if self.steps==True:
				print "Return to central position ?"
				raw_input()

			joint_cmd = self.move_group_gl_head.get_current_joint_values()
			joint_cmd[0] = 0.0
			joint_cmd[1] = 0.0
			joint_cmd[2:9] = self.joints_left_relax[:]
			joint_cmd[16] = 0.0
			joint_cmd[17] = 0.0
			self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "turn_torso_relax_left_arm")

		# BACK TO INITIAL POSE
		self.move_initial_pose()
		# Publish signal to stop the recording of the cameras
		if mqtt_enable:
			self.mqtt_pub.publish(0)
		# Killing the two threads: data collection of the robot and ros publisher for end effector state
		self.talos_data_collecter.clean_shutdown()
		print "Talos data collecter shutdown complete"
		if shimmer_enable:
			self.shimmer_data_collecter.clean_shutdown()
			print "Shimmer data collecter shutdown complete"
		self.pub_ee_state.clean_shutdown()
		print "Publisher state shutdown complete"
	# ========================================================================

	def clean_shutdown(self):
		self.t2.stop()
		self.talos_data_collecter.clean_shutdown()
		if shimmer_enable:
			self.shimmer_data_collecter.clean_shutdown()
		self.pub_ee_state.clean_shutdown()




def main():
	try:
		print " ============================================================================= "
		print " ============================ PICK AND PLACE TEST ============================ "
		print " ============================================================================= "

		print " ------------------------------------------ "
		print " ==> Press 'Enter' to initialize "
		print " ------------------------------------------ "
		raw_input()
		PP_test = TalosPickPlace()

		#nb_test = 0
		#while nb_test < 5:
		print " ------------------------------------------------------ "
		print " ==> Press 'Enter' to launch pick and place "#- test %s" % nb_test
		print " ------------------------------------------------------ "
		raw_input()
		PP_test.pick_and_place_all()
		#PP_test.load_scene()
		#PP_test.move_torso(0, [PP_test.const.TORSO_TURN,0])
		#	nb_test = nb_test + 1


	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	    return

if __name__ == '__main__':
  main()
