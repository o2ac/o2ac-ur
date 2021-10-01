#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from o2ac_flexbe_states.playback_sequence import PlayBackActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 11 2021
@author: Cristian Beltran
'''
class demoSM(Behavior):
	'''
	Playback sequences
	'''


	def __init__(self):
		super(demoSM, self).__init__()
		self.name = 'demo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		playback_sequence_name1 = "bearing_orient_down"
		playback_sequence_name2 = "bearing_move_to_taskboard"
		# x:30 y:365, x:505 y:140
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('playback sequence',
										PlayBackActionState(sequence_name=playback_sequence_name1),
										transitions={'success': 'bearing to taskboard', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

			# x:104 y:178
			OperatableStateMachine.add('bearing to taskboard',
										PlayBackActionState(sequence_name=playback_sequence_name2),
										transitions={'success': 'finished', 'error': 'failed'},
										autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
