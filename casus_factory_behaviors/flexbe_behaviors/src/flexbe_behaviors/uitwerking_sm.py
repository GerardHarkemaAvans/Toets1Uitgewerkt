#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from casus_factory_flexbe_states.compute_grasp_state import ComputeGraspState
from casus_factory_flexbe_states.detect_part_camera_state import DetectPartCameraState
from casus_factory_flexbe_states.moveit_cartesian_to_joints_dyn_state import MoveitCartesianToJointsDynState
from casus_factory_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_manipulation_states.moveit_to_joints_dyn_state import MoveitToJointsDynState
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit as flexbe_manipulation_states__SrdfStateToMoveit
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D

# [/MANUAL_IMPORT]


'''
Created on 22 februari 2020
@author: Gerard Harkema
'''
class UitwerkingSM(Behavior):
	'''
	Uitwerking van Flexbe machine
	'''


	def __init__(self):
		super(UitwerkingSM, self).__init__()
		self.name = 'Uitwerking'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]

		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		names = ['robot1_shoulder_pan_joint', 'robot1_shoulder_lift_joint', 'robot1_elbow_joint', 'robot1_wrist_1_joint', 'robot1_wrist_2_joint', 'robot1_wrist_3_joint']
		move_group = 'robot1'
		gripper = "vacuum_gripper1_suction_cup"
		part = 'gear_part'
		part_height = 0.015
		action_topic = '/move_group'
		# x:1002 y:538, x:594 y:345
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.part_pose = []
		_state_machine.userdata.pick_configuration = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:63
			OperatableStateMachine.add('Move Robot Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Home', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Move Robot to Pre Grasp', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:647 y:66
			OperatableStateMachine.add('Compute pick',
										ComputeGraspState(group=move_group, offset=part_height, joint_names=names, tool_link=gripper, rotation=3.14),
										transitions={'continue': 'Move to part cartesian', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose', 'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:968 y:319
			OperatableStateMachine.add('Deactivate Gripper',
										VacuumGripperControlState(enable=False, service_name='/gripper1/control'),
										transitions={'continue': 'Move Robot back Home', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:478 y:64
			OperatableStateMachine.add('Detect Part Camera',
										DetectPartCameraState(ref_frame='world', camera_topic='/casus/logical_camera_1', camera_frame='logical_camera_1_frame', part=part),
										transitions={'continue': 'Compute pick', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pose': 'part_pose'})

			# x:963 y:139
			OperatableStateMachine.add('Move Robot Back to Pre Grasp',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='PreGrasp', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Move Robot to Drop', 'planning_failed': 'failed', 'control_failed': 'WaiRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:965 y:397
			OperatableStateMachine.add('Move Robot back Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Home', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:967 y:228
			OperatableStateMachine.add('Move Robot to Drop',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Drop', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Deactivate Gripper', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:273 y:63
			OperatableStateMachine.add('Move Robot to Pre Grasp',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='PreGrasp', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Detect Part Camera', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:789 y:105
			OperatableStateMachine.add('Move to Part',
										MoveitToJointsDynState(move_group='robot1', action_topic='/move_group'),
										transitions={'reached': 'Activate Gripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:776 y:14
			OperatableStateMachine.add('Move to part cartesian',
										MoveitCartesianToJointsDynState(move_group='robot1', offset=0.0, tool_link=gripper, action_topic='/move_group'),
										transitions={'reached': 'Activate Gripper', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_values': 'pick_configuration', 'joint_names': 'joint_names'})

			# x:1175 y:49
			OperatableStateMachine.add('WachtEven',
										WaitState(wait_time=1),
										transitions={'done': 'Move Robot Back to Pre Grasp'},
										autonomy={'done': Autonomy.Off})

			# x:1170 y:141
			OperatableStateMachine.add('WaiRetry',
										WaitState(wait_time=2),
										transitions={'done': 'Move Robot Back to Pre Grasp'},
										autonomy={'done': Autonomy.Off})

			# x:960 y:74
			OperatableStateMachine.add('Activate Gripper',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'WachtEven', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
