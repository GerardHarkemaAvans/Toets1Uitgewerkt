#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from casus_factory_flexbe_states.dummy_state import DummyState
from casus_factory_flexbe_states.vacuum_gripper_control_state import VacuumGripperControlState
from flexbe_manipulation_states.srdf_state_to_moveit import SrdfStateToMoveit as flexbe_manipulation_states__SrdfStateToMoveit
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import Pose2D

# [/MANUAL_IMPORT]


'''
Created on 31 maart 2021
@author: Gerard Harkema
'''
class casus6_assignmentSM(Behavior):
	'''
	Flexbe machine
	'''


	def __init__(self):
		super(casus6_assignmentSM, self).__init__()
		self.name = 'casus6_assignment'

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
		robot_name = ''
		# x:1037 y:645, x:441 y:399
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:63
			OperatableStateMachine.add('Move R1 Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Home', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Move R1 PreGrasp', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:965 y:469
			OperatableStateMachine.add('Deactivate Gripper 1',
										VacuumGripperControlState(enable=False, service_name='/gripper1/control'),
										transitions={'continue': 'Move R1 back Home', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:955 y:319
			OperatableStateMachine.add('Move R1 Back to PreGrasp',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='PreGrasp', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Move R2 to drop', 'planning_failed': 'failed', 'control_failed': 'WaiRetry', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:273 y:63
			OperatableStateMachine.add('Move R1 PreGrasp',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='PreGrasp', move_group=move_group, action_topic='/move_group', robot_name=''),
										transitions={'reached': 'ToDo1', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:966 y:551
			OperatableStateMachine.add('Move R1 back Home',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Home', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:963 y:394
			OperatableStateMachine.add('Move R2 to drop',
										flexbe_manipulation_states__SrdfStateToMoveit(config_name='Drop', move_group=move_group, action_topic=action_topic, robot_name=''),
										transitions={'reached': 'Deactivate Gripper 1', 'planning_failed': 'failed', 'control_failed': 'failed', 'param_error': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off, 'param_error': Autonomy.Off},
										remapping={'config_name': 'config_name', 'move_group': 'move_group', 'robot_name': 'robot_name', 'action_topic': 'action_topic', 'joint_values': 'joint_values', 'joint_names': 'joint_names'})

			# x:471 y:66
			OperatableStateMachine.add('ToDo1',
										DummyState(active_output=True),
										transitions={'pass': 'ToDo2', 'failed': 'failed'},
										autonomy={'pass': Autonomy.Off, 'failed': Autonomy.Off})

			# x:634 y:66
			OperatableStateMachine.add('ToDo2',
										DummyState(active_output=True),
										transitions={'pass': 'ToDo3', 'failed': 'failed'},
										autonomy={'pass': Autonomy.Off, 'failed': Autonomy.Off})

			# x:796 y:66
			OperatableStateMachine.add('ToDo3',
										DummyState(active_output=True),
										transitions={'pass': 'Activate Gripper 1', 'failed': 'failed'},
										autonomy={'pass': Autonomy.Off, 'failed': Autonomy.Off})

			# x:999 y:186
			OperatableStateMachine.add('WachtEven',
										WaitState(wait_time=1),
										transitions={'done': 'Move R1 Back to PreGrasp'},
										autonomy={'done': Autonomy.Off})

			# x:1164 y:316
			OperatableStateMachine.add('WaiRetry',
										WaitState(wait_time=2),
										transitions={'done': 'Move R1 Back to PreGrasp'},
										autonomy={'done': Autonomy.Off})

			# x:960 y:74
			OperatableStateMachine.add('Activate Gripper 1',
										VacuumGripperControlState(enable=True, service_name='/gripper1/control'),
										transitions={'continue': 'WachtEven', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]

	# [/MANUAL_FUNC]
