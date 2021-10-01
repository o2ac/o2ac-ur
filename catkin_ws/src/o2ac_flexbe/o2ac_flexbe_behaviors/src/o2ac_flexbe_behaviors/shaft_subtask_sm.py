#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from o2ac_flexbe_states.fasten import FastenActionState
from o2ac_flexbe_states.insertion import InsertActionState
from o2ac_flexbe_states.move_to import MoveToActionState
from o2ac_flexbe_states.orient import OrientActionState
from o2ac_flexbe_states.pick import PickActionState
from o2ac_flexbe_states.playback_sequence import PlayBackActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 29 2021
@author: Cristian Beltran
'''
class shaftsubtaskSM(Behavior):
	'''
	assembly subtask c2
Assembly shaft with cap and m4 screw then insert it into the bearing
	'''


	def __init__(self):
		super(shaftsubtaskSM, self).__init__()
		self.name = 'shaft subtask'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		task = "assembly"
		shaft = "shaft"
		go_to_insertion_seq = "shaft_go_to_bearing_assembly"
		end_cap = "end_cap"
		a_bot = "a_bot"
		b_bot = "b_bot"
		# x:687 y:392, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:425 y:66, x:130 y:365
		_sm_pick_and_orient_shaft_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_pick_and_orient_shaft_0:
			# x:61 y:40
			OperatableStateMachine.add('Pick shaft',
										PickActionState(robot_name=b_bot, object_name=shaft),
										transitions={'success': 'Orient shaft', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:246 y:40
			OperatableStateMachine.add('Orient shaft',
										OrientActionState(robot_name=b_bot, task_name=task, object_name=shaft),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		# x:450 y:54, x:180 y:296
		_sm_pick_and_orient_end_cap_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_pick_and_orient_end_cap_1:
			# x:93 y:43
			OperatableStateMachine.add('Pick end cap',
										PickActionState(robot_name=a_bot, object_name=end_cap),
										transitions={'success': 'Orient end cap', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:240 y:42
			OperatableStateMachine.add('Orient end cap',
										OrientActionState(robot_name=a_bot, task_name=task, object_name=end_cap),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		# x:520 y:303, x:130 y:365
		_sm_insert_shaft_2 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_insert_shaft_2:
			# x:52 y:47
			OperatableStateMachine.add('Move shaft to pre-insertion pose',
										PlayBackActionState(sequence_name=go_to_insertion_seq),
										transitions={'success': 'Insert shaft', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:365 y:124
			OperatableStateMachine.add('Insert shaft',
										InsertActionState(robot_name=b_bot, task_name=task, object_name=shaft),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		# x:631 y:539, x:46 y:444
		_sm_insert_end_cap_into_shaft_3 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_insert_end_cap_into_shaft_3:
			# x:30 y:40
			OperatableStateMachine.add('Move shaft to insertion pose',
										MoveToActionState(robot_name=b_bot, target_pose=[0.0, 0, 0.2, 0, 0, -90.0], pose_type="cartesian", frame_id="tray_center", target_named_pose="", motion_planner="ompl"),
										transitions={'success': 'Move end cap to pre-insertion pose', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:339 y:195
			OperatableStateMachine.add('Insert end cap',
										InsertActionState(robot_name=a_bot, task_name=task, object_name=end_cap),
										transitions={'success': 'Fasten end cap', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:240 y:41
			OperatableStateMachine.add('Move end cap to pre-insertion pose',
										MoveToActionState(robot_name=a_bot, target_pose=[-0.002, -0.001, 0.25, -180, 90, -90], pose_type="cartesian", frame_id="tray_center", target_named_pose="", motion_planner="ompl"),
										transitions={'success': 'Insert end cap', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:551 y:298
			OperatableStateMachine.add('Fasten end cap',
										FastenActionState(task_name=task, object_name=end_cap),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})



		with _state_machine:
			# x:78 y:33
			OperatableStateMachine.add('Pick and orient end_cap',
										_sm_pick_and_orient_end_cap_1,
										transitions={'finished': 'Pick and orient shaft', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:628 y:277
			OperatableStateMachine.add('Insert shaft',
										_sm_insert_shaft_2,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:271 y:117
			OperatableStateMachine.add('Pick and orient shaft',
										_sm_pick_and_orient_shaft_0,
										transitions={'finished': 'Insert end cap into shaft', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:460 y:186
			OperatableStateMachine.add('Insert end cap into shaft',
										_sm_insert_end_cap_into_shaft_3,
										transitions={'finished': 'Insert shaft', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
