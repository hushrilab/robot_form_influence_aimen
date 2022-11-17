#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------
""" Class built to contain all common functions for both tasks for TIAGo """ 
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
from math import ceil
# ---------------------------------------------------------------------------

class TalosGl(object):
	"""TalosGl"""

	# ============================ INITIALIZATION ============================
	def __init__(self, const):
		# const is the file of constants that will be imported either for pickplace or handover task

		super(TalosGl, self).__init__()

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('talos_task', anonymous=True, log_level=rospy.DEBUG)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface() # experiment environment

		# Definition of the distinct move groups that will be needed

		group_name_gl = "both_arms_torso"
		self.move_group_gl = moveit_commander.MoveGroupCommander(group_name_gl,wait_for_servers=10.)
		#self.move_group_gl = moveit_commander.MoveGroupCommander(group_name_gl)
		
                self.move_group_gl.set_planning_time(const.PLANNING_TIME)
		self.move_group_gl.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_head = "head"
		self.move_group_head = moveit_commander.MoveGroupCommander(group_name_head)
		self.move_group_head.set_planning_time(const.PLANNING_TIME)
		self.move_group_head.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_leg_l = "left_leg"
		self.move_group_leg_l = moveit_commander.MoveGroupCommander(group_name_leg_l)
		self.move_group_leg_l.set_planning_time(const.PLANNING_TIME)
		self.move_group_leg_l.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_leg_r = "right_leg"
		self.move_group_leg_r = moveit_commander.MoveGroupCommander(group_name_leg_r)
		self.move_group_leg_r.set_planning_time(const.PLANNING_TIME)
		self.move_group_leg_r.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_torso_head = "torso_head"
		self.move_group_torso_head = moveit_commander.MoveGroupCommander(group_name_torso_head)
		self.move_group_torso_head.set_planning_time(const.PLANNING_TIME)
		self.move_group_torso_head.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		group_name_gl_head = "both_arms_torso_head"
		self.move_group_gl_head = moveit_commander.MoveGroupCommander(group_name_gl_head)
		self.move_group_gl_head.set_planning_time(const.PLANNING_TIME)
		self.move_group_gl_head.set_num_planning_attempts(const.PLANNING_ATTEMPTS)

		# Rotation matrix btwn right wrist init and base link
		self.R_base_wrist_r = np.array([[-1.0,0.0,0.0,0.0],
										[0.0,-1.0,0.0,0.0],
										[0.0,0.0,1.0,0.0],
										[0.0,0.0,0.0,1.0]])
		# Constraints matrix to have the right orientation of the wrist when grasping
		self.R_constraints_wrist_r = np.array([[0.0,-1.0,0.0,0.0],
											[0.0,0.0,1.0,0.0],
											[-1.0,0.0,0.0,0.0],
											[0.0,0.0,0.0,1.0]])
		# Rotation matrix btwn left wrist init and base link
		self.R_base_wrist_l = np.array([[1.0,0.0,0.0,0.0],
										[0.0,1.0,0.0,0.0],
										[0.0,0.0,1.0,0.0],
										[0.0,0.0,0.0,1.0]])
		# Constraints matrix to have the right orientation of the wrist when placing
		self.R_constraints_wrist_l = np.array([[0.0,-1.0,0.0,0.0],
											[0.0,0.0,-1.0,0.0],
											[1.0,0.0,0.0,0.0],
											[0.0,0.0,0.0,1.0]])

		# Move TALOS to its definded initial pose
		self.move_initial_pose()
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                                   moveit_msgs.msg.DisplayTrajectory,
	                                                   queue_size=20)

		self.robot = robot
		self.scene = scene
		self.obj_to_grasp = const.OBJ_NAMES_GRASP
		self.obj_scene = const.OBJ_NAMES_SCENE
		self.steps = False
	# ========================================================================


	# ============================= RESET SCENE ==============================
	def reset_scene(self):
		# Remove everything from the environment
		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			for i in range(self.const.NB_GROUPS):
				self.scene.remove_attached_object(self.eef_links[i], name=obj_name)
			self.scene.remove_world_object(obj_name)
		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
				self.scene.remove_world_object(obj_name_scene)
	# ========================================================================


	# ============================== LOAD SCENE ==============================
	def load_scene(self):
		""" Load the experiment environment on moveit"""
		self.reset_scene()
		rospy.sleep(2)

		# OBJECTS OF THE ENVIRONMENT
		for obj_name_scene in self.obj_scene:
			p = PoseStamped()
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position.x = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("x")
			p.pose.position.y = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("y")
			p.pose.position.z = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("z")
			quaternion = quaternion_from_euler(self.const.OBJ_POS_SCENE.get(obj_name_scene).get("roll"),self.const.OBJ_POS_SCENE.get(obj_name_scene).get("pitch"),self.const.OBJ_POS_SCENE.get(obj_name_scene).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

			# ADD the objects to the scene depending on their type
			obj_type = self.const.OBJ_POS_SCENE.get(obj_name_scene).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("h"), self.const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("n"), self.const.OBJ_POS_SCENE.get(obj_name_scene).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name_scene, p, self.const.OBJ_POS_SCENE.get(obj_name_scene).get("r"))
			else:
				print "[ERROR] Type of object %s to add not supported." % obj_type

		# OBJECTS TO GRASP
		for obj_name in self.obj_to_grasp:
			p.header.frame_id = self.robot.get_planning_frame()
			p.pose.position.x = self.const.OBJ_POS_GRASP.get(obj_name).get("x")
			p.pose.position.y = self.const.OBJ_POS_GRASP.get(obj_name).get("y")
			p.pose.position.z = self.const.OBJ_POS_GRASP.get(obj_name).get("z")
			quaternion = quaternion_from_euler(self.const.OBJ_POS_GRASP.get(obj_name).get("roll"),self.const.OBJ_POS_GRASP.get(obj_name).get("pitch"),self.const.OBJ_POS_GRASP.get(obj_name).get("yaw"))
			p.pose.orientation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
			# ADD the objects to the scene depending on their type
			obj_type = self.const.OBJ_POS_GRASP.get(obj_name).get("type")
			if obj_type == "BOX":
				self.scene.add_box(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("dim"))
			elif obj_type == "CYLINDER":
				self.scene.add_cylinder(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("h"), self.const.OBJ_POS_GRASP.get(obj_name).get("r"))
			elif obj_type == "PLANE":
				self.scene.add_plane(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("n"), self.const.OBJ_POS_GRASP.get(obj_name).get("offset"))
			elif obj_type == "SPHERE":
				self.scene.add_sphere(obj_name, p, self.const.OBJ_POS_GRASP.get(obj_name).get("r"))
			else:
				print "[ERROR] Type of object %s to add not supported." % obj_type
		print "Scene set up done."
		print ""
	# ========================================================================


	# ============================= GET DIM OBJ ==============================
	def get_dim_obj(self, obj_name, axis):
		""" Get dimension of an object along x, y or z """
		obj_type = self.const.OBJ_POS_GRASP.get(obj_name).get("type")

		if obj_type == "BOX":
			dim = self.const.OBJ_POS_GRASP.get(obj_name).get("dim")[axis]
		elif obj_type == "CYLINDER":
			if axis == 2:
				dim = self.const.OBJ_POS_GRASP.get(obj_name).get("h")
			else:
				dim = self.const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		elif obj_type == "SPHERE":
			dim = self.const.OBJ_POS_GRASP.get(obj_name).get("r") * 2
		else:
			print "[ERROR] Type of object %s to grasp not supported." % obj_type

		return dim
	# ========================================================================

	# ============================= OPEN GRIPPER =============================
	def open_gripper(self, id_group):
		""" Open gripper end effector according to id group  0 = right ee, 1 = left_ee"""
		gripper_group = self.gripper_groups[id_group]
		gripper_group.set_joint_value_target([self.const.JOINT_OPEN_GRIPPER])
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
	# ========================================================================


	# ============================ CLOSE GRIPPER =============================
	def close_gripper(self, id_group):
		""" Open gripper end effector according to id group  0 = right ee, 1 = left_ee"""
		gripper_group = self.gripper_groups[id_group]
		gripper_group.set_joint_value_target([self.const.JOINT_CLOSE_GRIPPER])
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


	# ============================= MOVE INITIAL =============================
	def move_initial_pose(self):
		self.move_head([0.0,0.0])
		self.move_group_gl.set_named_target("home_grasp_both_arms_torso");
		plan = self.move_group_gl.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to move to initial pose.")
			sys.exit(1)
		else :
			print("Executing plan found to reach initial pose...")
			self.move_group_gl.execute(plan)
			self.move_group_gl.stop()
			self.move_group_gl.clear_pose_targets()
			print "Initial pose reached."
			print ""

		#Add legs
		# pose_leg_l = self.move_group_leg_l.get_current_pose()
		# pose_leg_r = self.move_group_leg_r.get_current_pose()
		# pose_leg_l.pose.position.x = pose_leg_l.pose.position.x - 0.05
		# pose_leg_l.pose.position.z = pose_leg_l.pose.position.z + 0.05
		# pose_leg_r.pose.position.x = pose_leg_r.pose.position.x - 0.05
		# pose_leg_r.pose.position.z = pose_leg_r.pose.position.z + 0.05
		# self.move_group_leg_l.set_pose_target(pose_leg_l)
		# self.move_group_leg_r.set_pose_target(pose_leg_r)

		#Add legs second method
		# joints_leg_l = [0.0, 0.009, -0.25, 0.676, -0.29, -0.009]
		# joints_leg_r = [0.0, -0.009, -0.25, 0.676, -0.29, 0.009]
		# self.move_group_leg_l.set_joint_value_target(joints_leg_l)
		# self.move_group_leg_r.set_joint_value_target(joints_leg_r)

		# plan = self.move_group_leg_l.plan()
		# if len(plan.joint_trajectory.points) == 0 :
		# 	print("[ERROR] No plan found to move to initial pose left leg.")
		# 	sys.exit(1)
		# else :
		# 	print("Executing plan found to reach initial pose...")
		# 	self.move_group_leg_l.execute(plan)
		# 	self.move_group_leg_l.stop()
		# 	self.move_group_leg_l.clear_pose_targets()
		# 	print "Initial pose reached left leg."
		# 	print ""
		# plan = self.move_group_leg_r.plan()
		# if len(plan.joint_trajectory.points) == 0 :
		# 	print("[ERROR] No plan found to move to initial pose right leg.")
		# 	sys.exit(1)
		# else :
		# 	print("Executing plan found to reach initial pose...")
		# 	self.move_group_leg_r.execute(plan)
		# 	self.move_group_leg_r.stop()
		# 	self.move_group_leg_r.clear_pose_targets()
		# 	print "Initial pose reached right leg."
		# 	print ""
	# ========================================================================


	# ============================= MOVE HEAD ===============================
	def move_head(self,joint_values):
		current_joints = self.move_group_head.get_current_joint_values()
		target_joints = current_joints
		target_joints[0] = joint_values[0]
		target_joints[1] = joint_values[1]
		self.move_group_head.set_joint_value_target(target_joints)
		plan = self.move_group_head.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print "[ERROR] No plan found to move head."
			print ""
			sys.exit(1)
		else :
			print "Executing plan found to move head."
			self.move_group_head.execute(plan)
			self.move_group_head.stop()
			self.move_group_head.clear_pose_targets()
			print "Position reached."
			print ""
	# ========================================================================


	# ============================== RELAX ARM ===============================
	def relax_arm(self, id_group):

		if id_group == 2:
			move_group = self.move_group_gl
		else :
			move_group = self.move_groups[id_group]
		move_group.set_named_target(self.const.RELAX_POS[id_group])
		plan = move_group.plan()
		if len(plan.joint_trajectory.points) == 0 :
			print("[ERROR] No plan found to relax arm.")
			sys.exit(1)
		else :
			print("Executing plan found to relax arm...")
			move_group.execute(plan)
			move_group.stop()
			move_group.clear_pose_targets()
		print "Relax position reached."
		print ""
	# ========================================================================


	# ======================== EXECUTE JOINT COMMAND =======================
	def execute_joints_cmd(self, move_group, joint_cmd, name):

		move_group.set_joint_value_target(joint_cmd)
		plan = move_group.plan()
		if len(plan.joint_trajectory.points) == 0:
				print "[ERROR] No plan found to: %s." % name
				sys.exit(1)
		else:
			print "Executing plan found to %s." % name
			move_group.execute(plan)
			move_group.stop()
			move_group.clear_pose_targets()
			print "Position reached."
			print ""

	# ========================================================================

	









