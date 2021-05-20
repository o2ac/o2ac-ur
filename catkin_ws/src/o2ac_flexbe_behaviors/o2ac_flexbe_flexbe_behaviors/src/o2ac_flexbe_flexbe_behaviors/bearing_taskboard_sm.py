#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_state import LogState
from o2ac_flexbe_flexbe_states.align_bearing_holes import AlignBearingHolesActionState
from o2ac_flexbe_flexbe_states.insertion import InsertActionState
from o2ac_flexbe_flexbe_states.pickup import PickUpActionState
from o2ac_flexbe_flexbe_states.playback_sequence import PlayBackActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 11 2021
@author: Cristian Beltran
'''
class bearingtaskboardSM(Behavior):
	'''
	demo for bearing subtask
	'''


	def __init__(self):
		super(bearingtaskboardSM, self).__init__()
		self.name = 'bearing taskboard'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		object_name = "bearing"
		seq1 = "bearing_orient"
		seq2 = "bearing_move_to_taskboard"
		valid_grasp_res = ["up", "down"]
		seq3 = "bearing_orient_down"
		task_name = "taskboard"
		log = "yay"
		# x:1111 y:423, x:160 y:552
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:77 y:45
			OperatableStateMachine.add('pick up bearing',
										PickUpActionState(object_name=object_name),
										transitions={'success': 'is bearing upside down', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'gripper_opening': 'gripper_opening'})

			# x:754 y:429
			OperatableStateMachine.add('Insert bearing',
										InsertActionState(object_name=object_name, task_name=task_name),
										transitions={'success': 'Align holes', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:294 y:84
			OperatableStateMachine.add('is bearing upside down',
										CheckConditionState(predicate=self.grasped_bearing_is_upside_down),
										transitions={'true': 'orient bearing', 'false': 'orient bearing down'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'gripper_opening'})

			# x:1113 y:587
			OperatableStateMachine.add('log',
										LogState(text=log, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:613 y:348
			OperatableStateMachine.add('move to taskboard',
										PlayBackActionState(sequence_name=seq2),
										transitions={'success': 'Insert bearing', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:453 y:240
			OperatableStateMachine.add('orient bearing',
										PlayBackActionState(sequence_name=seq1),
										transitions={'success': 'move to taskboard', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:562 y:113
			OperatableStateMachine.add('orient bearing down',
										PlayBackActionState(sequence_name=seq3),
										transitions={'success': 'move to taskboard', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:822 y:560
			OperatableStateMachine.add('Align holes',
										AlignBearingHolesActionState(task_name=task_name),
										transitions={'success': 'log', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def grasped_bearing_is_upside_down(self, grasp_opening):
		return grasp_opening > 0.045
	# [/MANUAL_FUNC]
