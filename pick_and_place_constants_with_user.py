#!/usr/bin/env python

#----------------------------------------------------------------------------
# Created By  : Julie Pivin-Bachler
# Created Date: 03/2022 - 08/2022
# ---------------------------------------------------------------------------

NB_GROUPS = 2

# ===== GENERAL - ROBOT AND PLANNING PARAMETERS =====
GROUP_NAME = "right_arm"
LARGE_GROUP_NAME = "right_arm_torso"
GRIPPER_NAME = "right_arm_gripper"
GRIPPER_GROUP = "gripper_right"
EEF = "gripper_right_base_link"
HOME_POS = "home_grasp_right_arm_torso"
RELAX_POS = ["home_grasp_right_arm","relax_both_arms_torso"]
DIST = 0.10 # estimated distance between tool and wrist link
JOINT_CLOSE_GRIPPER = -0.46 #-0.485
JOINT_OPEN_GRIPPER = 0.0
PLANNING_TIME = 10
PLANNING_ATTEMPTS = 50
SWITCH_POSITION = {
	"x" : 0.75,
	"y" : -0.08,
	"z" : 0.05, 
	"roll" : 0.0,
	"pitch" : 0.0,
	"yaw" : 0.0
}
TORSO_TURN = 1.2

# ===== SCENE ELEMENTS =====
OBJ_NAMES_SCENE = ["shelf1","shelf2","shelf3","shelf4"]
OBJ_POS_SCENE = {
	"shelf1" : {
		"type" : "BOX",
		"x" : -0.15,
		"y" : -1.0,
		"z" : -0.6, 
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.9)
	},
	"shelf2" : {
		"type" : "BOX",
		"x" : -0.15,
		"y" : 1.0,
		"z" : -0.6,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.9)
	},
	"shelf3" : {
		"type" : "BOX",
		"x" : 1.75,
		"y" : -1.0,
		"z" : -0.6, 
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.9)
	},
	"shelf4" : {
		"type" : "BOX",
		"x" : 1.75,
		"y" : 1.0,
		"z" : -0.6,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.65, 0.6, 0.9)
	}
}


# ===== ITEMS TO GRASP =====
OBJ_NAMES_GRASP = ["object1", "object2", "object3"]
OBJ_POS_GRASP = {
	"object1" : {
		"type" : "BOX",
		"x" : 0.05,
		"y" : -0.75,
		"z" : 0.05,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.055, 0.055, 0.4)
	},
	"object2" : {
		"type" : "BOX",
		"x" : -0.10,
		"y" : -0.75,
		"z" : 0.05,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.4)
	},
	"object3" : {
		"type" : "BOX",
		"x" : -0.25,
		"y" : -0.75,
		"z" : 0.05,
		"roll" : 0.0,
		"pitch" : 0.0,
		"yaw" : 0.0,
		"dim" : (0.05, 0.05, 0.4)
	}
}

# APPROACH DISTANCES - PICKING // INITIAL OBJECT POSE
APPROACH_PICK_X = 0.0
APPROACH_PICK_Y = 0.115
APPROACH_PICK_Z = 0.0

# RETREAT DISTANCES - PRE PLACING // INITIAL OBJECT POSE
RETREAT_X = 0.0
RETREAT_Y = 0.115
RETREAT_Z = 0.05

# APPROACH DISTANCES - PLACING // INITIAL OBJECT POSE
APPROACH_PLACE_X = -0.1
APPROACH_PLACE_Y = 0.15
APPROACH_PLACE_Z = 0.00

#GRIPPER ORIENTATION FOR SWITCH POINT
EULER_SWITCH = [0.0,1.5708,3.1415]
EULER_PICK = [1.5708,1.5708,3.1415]

#MARGINS
MARGIN_X = 0.0
MARGIN_Y = 0.01
MARGIN_Z = 0.0
