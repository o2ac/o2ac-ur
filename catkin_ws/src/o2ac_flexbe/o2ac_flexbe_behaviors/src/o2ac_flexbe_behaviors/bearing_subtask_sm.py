#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.log_state import LogState
from o2ac_flexbe_states.align_bearing_holes import AlignBearingHolesActionState
from o2ac_flexbe_states.fasten import FastenActionState
from o2ac_flexbe_states.insertion import InsertActionState
from o2ac_flexbe_states.orient import OrientActionState
from o2ac_flexbe_states.pick import PickActionState
from o2ac_flexbe_states.playback_sequence import PlayBackActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 11 2021
@author: Cristian Beltran
'''
class bearingsubtaskSM(Behavior):
	'''
	bearing subtask
	'''


	def __init__(self):
		super(bearingsubtaskSM, self).__init__()
		self.name = 'bearing subtask'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		object_name = "bearing"
		sequence1 = "bearing_move_to_taskboard"
		task_name = "taskboard"
		log = "Completed"
		# x:993 y:542, x:160 y:552
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:77 y:45
			OperatableStateMachine.add('pick bearing',
										PickActionState(object_name=object_name),
										transitions={'success': 'orient bearing', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:487 y:247
			OperatableStateMachine.add('Insert bearing',
										InsertActionState(object_name=object_name, task_name=task_name),
										transitions={'success': 'Align holes', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:817 y:383
			OperatableStateMachine.add('fasten bearing',
										FastenActionState(task_name=task_name, object_name=object_name),
										transitions={'success': 'log', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:971 y:396
			OperatableStateMachine.add('log',
										LogState(text=log, severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:370 y:187
			OperatableStateMachine.add('move to taskboard',
										PlayBackActionState(sequence_name=sequence1),
										transitions={'success': 'Insert bearing', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:236 y:106
			OperatableStateMachine.add('orient bearing',
										OrientActionState(task_name=task_name, object_name=object_name),
										transitions={'success': 'move to taskboard', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:626 y:319
			OperatableStateMachine.add('Align holes',
										AlignBearingHolesActionState(task_name=task_name),
										transitions={'success': 'fasten bearing', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def grasped_bearing_is_upside_down(self, grasp_opening):
		return grasp_opening > 0.045
	# [/MANUAL_FUNC]
