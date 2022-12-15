#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to contain all functions for handover task for TIAGo """ 
# ---------------------------------------------------------------------------
#                                  IMPORTS
# ---------------------------------------------------------------------------
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String, Duration
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation, CollisionObject
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_from_matrix
import pick_and_place_constants_with_user as const
#import pick_and_place_constants_with_user_45 as const
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from talos_gl import TalosGl
from ungrasp_button import UngraspButton
from publisher_ee_state import PublisherEeState
from collect_data_talos import CollectDataTalos
#from collect_data_shimmer import CollectDataShimmer
from mqtt_publisher import MqttPublisher
import threading
import os
import shlex, subprocess
# ---------------------------------------------------------------------------

class TalosHandOver(TalosGl):
	"""TalosHandOver"""

	# ============================ INITIALIZATION ============================
	def __init__(self):
		TalosGl.__init__(self, const)

		group_name = const.GROUP_NAME
		l_group_name = const.LARGE_GROUP_NAME
		gripper_name = const.GRIPPER_NAME
		move_group = moveit_commander.MoveGroupCommander(group_name)
		move_group.set_planning_time(const.PLANNING_TIME)
		move_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
		move_l_group = moveit_commander.MoveGroupCommander(l_group_name)
		move_l_group.set_planning_time(const.PLANNING_TIME)
		move_l_group.set_num_planning_attempts(const.PLANNING_ATTEMPTS)
		gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
		gripper_group.set_end_effector_link(const.EEF)
		touch_links = self.robot.get_link_names(group=gripper_name)
		for item in self.robot.get_link_names(group=const.GRIPPER_GROUP):
			if item not in touch_links:
				touch_links.append(item)
		eef_link = move_group.get_end_effector_link()

		self.move_group = move_group
		self.move_l_group = move_l_group
		self.gripper_group = gripper_group
		self.touch_links = touch_links
		self.eef_link = eef_link

		self.joints_right_relax = []
		#right = self.move_group.get_named_target_values(const.RELAX_POS[0])
		#for i in range(7):
		#	self.joints_right_relax.append(right.get("arm_right_" + str(i+1) + "_joint"))
		self.joints_right_relax = [0.0, -0.15, 0.4, -0.7, 0.0, 0.0, 0.0]
	
		self.change = False
		self.nb = 120
		self.l_forces = []
		self.force_mean = [0,0,0]
		self.force_sum = [0,0,0]
		self.force_mem = [0,0,0]
		self.baseline = 0.0

		self._srv = rospy.ServiceProxy('/compute_fk', moveit_msgs.srv.GetPositionFK)
		self._srv.wait_for_service()

		self.const = const

		self.pub_ee_state = PublisherEeState(self.move_group, moveit_commander.MoveGroupCommander("left_arm"))
		self.talos_data_collecter = CollectDataTalos("handover")
#		self.shimmer_data_collecter = CollectDataShimmer()

		# Create one thread for publisher
		t1 = threading.Thread(name="ee", target=self.pub_ee_state.ee_state_talker, args=())
		t1.start()

		# Initialization of the mqtt publisher for the recording of the cameras
		self.mqtt_pub = MqttPublisher()

		rospy.on_shutdown(self.clean_shutdown)

		print "Initialization done from Talos HANDOVER task."
		print ""
	# ========================================================================


	# ============================= RESET SCENE ==============================
	def reset_scene(self):

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			self.scene.remove_attached_object(self.eef_link, name=obj_name)
			self.scene.remove_world_object(obj_name)
		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
				self.scene.remove_world_object(obj_name_scene)
	# ========================================================================

	def command_button(self):
		self.open_gripper()
		self.ungrasp_button.root.destroy()

	# ========================================================================
	def callback_wrist_sensor(self,data):
		
		force = [data.wrench.force.x, data.wrench.force.y, data.wrench.force.z]
		if len(self.l_forces) == self.nb: # list full
			self.l_forces.pop(0) # remove first element
		self.l_forces.append(force)
		
		# mean value
		for i in range(len(self.l_forces)):
			for j in range(len(self.force_mean)):
				self.force_mean[j] = self.force_mean[j] + self.l_forces[i][j]
		for k in range(len(self.force_mean)):
			self.force_mean[k] = self.force_mean[k]/len(self.l_forces)
		#rospy.loginfo("Mean force vector:\n%s",self.force_mean)
		if len(self.l_forces) < self.nb:
			self.baseline = self.baseline + self.force_mean[2]
		else:
			rospy.loginfo("Mean force vector:\n%s",self.force_mean)
			print "Baseline: %s" % (self.baseline/self.nb)
		if self.force_mem != [0,0,0] and len(self.l_forces) == self.nb and abs(self.force_mem[2] - self.baseline/self.nb) > 5.0: #TO CHANGE
			print "User has grasped the object : %s" % self.force_mean[2]
			self.change = True
		else:
			self.force_mem = self.force_mean		
	# ========================================================================

        
	# ========================================================================        
	def listener(self):
		self.sub = rospy.Subscriber("/right_wrist_ft", WrenchStamped, self.callback_wrist_sensor)
	# ========================================================================


	# ========================================================================
	def idle(self):
		#function to wait
		return ""
	# ========================================================================


	# ========================== FORWARD KINEMATICS ==========================
	def fk(self, group, joint_values):

		fk_request = moveit_msgs.srv.GetPositionFKRequest()
		fk_request.fk_link_names = [group.get_end_effector_link()]
		fk_request.robot_state.joint_state.name = group.get_active_joints()
		fk_request.robot_state.joint_state.position = joint_values
		fk_request.header.frame_id = self.robot.get_planning_frame()
		fk_result = self._srv.call(fk_request)
		return fk_result.pose_stamped[0].pose
	# ========================================================================


	# ========================= MOVE TO SWITCH POINT =========================
	def move_handover_point(self, obj_to_grasp):

		# Get the torso to initial position
		joint_val = self.move_l_group.get_current_joint_values()
		joint_val[0:2] = [0.0,0.0] # turn torso
		goal_pose = self.fk(self.move_l_group, joint_val)
		quaternion = quaternion_from_euler(const.EULER_SWITCH[0],const.EULER_SWITCH[1],const.EULER_SWITCH[2])
		#goal_pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]) # wrist natural
		#success = self.move_attempt(self.move_l_group, goal_pose, "turn_torso_wrist_natural_pose")
		self.move_torso([0.0,0.0])
		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = const.SWITCH_POSITION.get("x") - const.DIST
		p.pose.position.y = const.SWITCH_POSITION.get("y") 
		p.pose.position.z = const.SWITCH_POSITION.get("z")
		p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

		# MOVES TO SWITCH POINT & TURN HEAD
		if self.steps==True:
			print "Handover pose ?"
			raw_input()

		joints_right = self.get_joints_from_pose(self.move_l_group, p,"move_right_to_switch_point")
		joint_cmd = self.move_group_gl_head.get_current_joint_values()
		joint_cmd[0:2] = joints_right[0:2]
		joint_cmd[9:16] = joints_right[2:]
		joint_cmd[16] = 0.1
		joint_cmd[17] = 0.1
		self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "move_right_to_switch_turn_head")

		# UNCOMMENT FOR LISTENNER INPUT - APPLY FORCES AND ALL
		# self.change = False
		# self.baseline = 0.0
		# self.l_forces = []
		# self.listener()
		# #UNGRASP OBJECT WHEN USER HAS IT
		# while self.change == False:
		# 	self.idle()

		# self.sub.unregister()
		
		#self.ungrasp_button = UngraspButton(self.command_button)
		#self.ungrasp_button.launch()
		print("Hint enter when user has grasped object")
		raw_input()
		#print "Open gripper ?" #TO CHANGE WITH BUTTON
		#raw_input()

		#self.open_gripper()
		self.scene.remove_attached_object(self.move_group.get_end_effector_link(), name=obj_to_grasp)
		self.scene.remove_attached_object(self.eef_link, name=obj_to_grasp)
		rospy.sleep(2)
		print "User has taken object"
		self.scene.remove_world_object(obj_to_grasp)

		#MOVING AWAY & TURN HEAD
		if self.steps==True:
			print "Retreat pose ?"
			raw_input()
		p.pose.position.x = p.pose.position.x + const.APPROACH_PLACE_X
		joints_right = self.get_joints_from_pose(self.move_l_group, p,"move_away_from_switch_point")
		joint_cmd = self.move_group_gl_head.get_current_joint_values()
		joint_cmd[0:2] = joints_right[0:2]
		joint_cmd[9:16] = joints_right[2:]
		joint_cmd[16] = 0.0
		joint_cmd[17] = 0.0
		self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "move_right_to_switch_turn_head")

		if self.steps==True:
			print "Return to central pose ?"
			raw_input()
		#TURN TORSO + RELAX ARM
		joint_cmd = self.move_l_group.get_current_joint_values()
		joint_cmd[0] = 0.0
		joint_cmd[1] = 0.0
		joint_cmd[2:] = self.joints_right_relax
		self.execute_joints_cmd(self.move_l_group, joint_cmd, "turn_torso_relax_right")

		return p.pose
	# ========================================================================


	# ============================= OPEN GRIPPER =============================
	def open_gripper(self, button=False):

		gripper_group = self.gripper_group
		gripper_group.set_joint_value_target([const.JOINT_OPEN_GRIPPER])
		plan = gripper_group.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to open the gripper.")
			sys.exit(1)
		else :
			print("Executing plan found to open the gripper...")
			gripper_group.execute(plan)
			gripper_group.stop()
			gripper_group.clear_pose_targets()
			print "Gripper opened."
			print ""
		if button == True:
			self.ungrasp_button.root.destroy()
	# ========================================================================


	# ============================ CLOSE GRIPPER =============================
	def close_gripper(self):

		gripper_group = self.gripper_group
		gripper_group.set_joint_value_target([const.JOINT_CLOSE_GRIPPER])
		plan = gripper_group.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to close the gripper.")
			sys.exit(1)
		else :
			print("Executing plan found to close the gripper...")
			gripper_group.execute(plan)
			gripper_group.stop()
			gripper_group.clear_pose_targets()
			print "Gripper closed."
			print ""
	# ========================================================================


	# ============================= PICK PIPELINE ============================
	def pick(self, obj_to_grasp):

		move_group = self.move_group
		move_l_group = self.move_l_group
		eef_link = self.eef_link
		touch_links = self.touch_links
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
		obj_pose_base.pose.position.z = obj_pose_base.pose.position.z - dim_z/4
		# change ref frame from base to wrist
		obj_pose_wrist = np.dot(self.R_base_wrist_r,np.array([obj_pose_base.pose.position.x,obj_pose_base.pose.position.y,obj_pose_base.pose.position.z,1]))
		obj_pose_wrist = np.dot(obj_pose_wrist,rot_obj)
		#add in approach in wrist frame
		dim_y = self.get_dim_obj(obj_to_grasp,1)	
		obj_pose_wrist[0] = obj_pose_wrist[0] - self.const.APPROACH_PICK_X - self.const.MARGIN_X	
		obj_pose_wrist[1] = obj_pose_wrist[1] - self.const.APPROACH_PICK_Y - self.const.MARGIN_Y - self.const.DIST - dim_y/2
		obj_pose_wrist[2] = obj_pose_wrist[2] +  self.const.APPROACH_PICK_Z + self.const.MARGIN_Z 
		obj_pose_wrist = np.dot(rot_obj,obj_pose_wrist)
		# change ref frame from wrist to base
		obj_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), obj_pose_wrist)
		# define goal pose in base frame
		obj_pose_base.pose.position.x = obj_pose_base_v[0]
		obj_pose_base.pose.position.y = obj_pose_base_v[1]
		obj_pose_base.pose.position.z = obj_pose_base_v[2]
		
		success = self.move_attempt(move_group, obj_pose_base, "grasp_approach_pose")
		#if success == False:
		#	success = self.move_attempt(move_l_group, obj_pose_base, "grasp_approach_pose", True)
		
		# OPEN GRIPPER
		if self.steps==True:
			print "Open gripper ?"
			raw_input()
		self.open_gripper()

		if self.steps==True:
			print "Grasp pose ?"
			raw_input()

		# MOVE TO GRASP POSE
		grasp_approach_pose= obj_pose_base
		grasp_pose_base = grasp_approach_pose
		# change ref frame from base to wrist
		grasp_pose_wrist = np.dot(self.R_base_wrist_r,np.array([grasp_pose_base.pose.position.x,grasp_pose_base.pose.position.y,grasp_pose_base.pose.position.z,1]))
		grasp_pose_wrist = np.dot(grasp_pose_wrist,rot_obj)
		grasp_pose_wrist[0] = grasp_pose_wrist[0] + self.const.APPROACH_PICK_X
		grasp_pose_wrist[1] = grasp_pose_wrist[1] + self.const.APPROACH_PICK_Y
		grasp_pose_wrist[2] = grasp_pose_wrist[2] -  self.const.APPROACH_PICK_Z
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
		self.close_gripper()

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
		pick_retreat_pose_wrist[0] = pick_retreat_pose_wrist[0] - self.const.RETREAT_X
		pick_retreat_pose_wrist[1] = pick_retreat_pose_wrist[1] - self.const.RETREAT_Y
		pick_retreat_pose_wrist[2] = pick_retreat_pose_wrist[2] +  self.const.RETREAT_Z
		pick_retreat_pose_wrist = np.dot(rot_obj,pick_retreat_pose_wrist)
		# change ref frame from wrist to base
		pick_retreat_pose_base_v = np.dot(np.linalg.inv(self.R_base_wrist_r), pick_retreat_pose_wrist)
		# define goal pose in base frame
		pick_retreat_pose_base.pose.position.x = pick_retreat_pose_base_v[0]
		pick_retreat_pose_base.pose.position.y = pick_retreat_pose_base_v[1]
		pick_retreat_pose_base.pose.position.z = pick_retreat_pose_base_v[2]
		#success = self.move_attempt(move_l_group, pick_retreat_pose_base, "pick_retreat_pose")
		print "Pick pipeline done."
		print ""

		return pick_retreat_pose_base
	# ========================================================================


	# =========================== CHECK PLAN IS OK ===========================
	def check_plan(self, group, plan, name, second_attempt=False):
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
			if j1 < -0.4:
				print "Plan pb for 1st joint r"
				return False
			if j2 < -1.25:
				print "Plan pb for 2nd joint r"
				return False
			
		print "Plan checked!"
		return True

	# ========================================================================


	# ======================== GET POSE JOINTS' VALUES =======================
	def get_joints_from_pose(self, group, pose, name):

		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 10:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(group,plan,name)
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
	def move_torso(self,joint_values):
		current_joints = self.move_l_group.get_current_joint_values()
		target_joints = current_joints
		target_joints[0] = joint_values[0]
		if len(joint_values) == 2 :
			target_joints[1] = joint_values[1]
		self.move_l_group.set_joint_value_target(target_joints)
		plan = self.move_l_group.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print "[ERROR] No plan found to turn torso."
			print ""
			sys.exit(1)
		else :
			print "Executing plan found to turn torso."
			self.move_l_group.execute(plan)
			self.move_l_group.stop()
			self.move_l_group.clear_pose_targets()
			print "Position reached."
			print ""
	# ========================================================================


	# ============================ ATTEMPT TO MOVE ===========================
	def move_attempt(self, group, pose, name, second_attempt=False):
		if second_attempt :
			print "2nd attempt..."
		group.set_pose_target(pose)
		is_ok = False
		nb_try = 0
		while is_ok == False and nb_try < 10:
			print "Move attempt - try number: %s" % nb_try
			plan = group.plan()
			is_ok = self.check_plan(group,plan,name)
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

	# ============================= PICK AND PLACE ===========================
	def pick_and_place_all(self):

		self.load_scene()
		rospy.sleep(2)

#		self.shimmer_data_collecter._dump_data()

		# Create one thread for data collecter
		t2 = threading.Thread(name="talos_data", target=self.talos_data_collecter.activate_thread, args=())
		t2.start()
	
		# Publish signal to start recording with both cameras simultaneously
		self.mqtt_pub.publish(1)

		for obj_to_grasp in self.obj_to_grasp:

			if self.steps==True:
				print "Face right table ?"
				raw_input()

			#TURN TORSO - HEAD + RELAX ARM
			joint_cmd = self.move_group_gl_head.get_current_joint_values()
			joint_cmd[0] = -self.const.TORSO_TURN
			joint_cmd[9:16] = self.joints_right_relax
			joint_cmd[16] = 0.2
			joint_cmd[17] = -0.2
			self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "turn_torso_relax_arms")

			if self.steps==True:
				print "Launch pick action ?"
				raw_input()
		 	last_pose = self.pick(obj_to_grasp)

		 	if self.steps==True:
		 		print "Launch handover action ?"
				raw_input()
		 	last_pose = self.move_handover_point(obj_to_grasp)

		 	if self.steps==True:
		 		print "Return to central position ?"
				raw_input()
		 	joint_cmd = self.move_group_gl_head.get_current_joint_values()
			joint_cmd[0] = 0.0
			joint_cmd[1] = 0.0
			joint_cmd[9:16] = self.joints_right_relax
			joint_cmd[16] = 0.0
			joint_cmd[17] = 0.0
			self.execute_joints_cmd(self.move_group_gl_head, joint_cmd, "turn_torso_relax_left_arm")

		# BACK TO INITIAL POSE
		self.move_initial_pose()
		# Publish signal to stop the recording of the cameras
		self.mqtt_pub.publish(0)
		# Killing the two threads: data collection of the robot and ros publisher for end effector state
		self.talos_data_collecter.clean_shutdown()
		print "Talos data collecter shutdown complete"
#		self.shimmer_data_collecter.clean_shutdown()
#		print "Shimmer data collecter shutdown complete"
		self.pub_ee_state.clean_shutdown()
		print "Publisher state shutdown complete"
	# ========================================================================

	def clean_shutdown(self):
		self.talos_data_collecter.clean_shutdown()
#		self.shimmer_data_collecter.clean_shutdown()
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
		HO_test = TalosHandOver()

		print " ------------------------------------------ "
		print " ==> Press 'Enter' to launch pick and place "
		print " ------------------------------------------ "
		raw_input()
		HO_test.pick_and_place_all()


	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
	    return

if __name__ == '__main__':
  main()
