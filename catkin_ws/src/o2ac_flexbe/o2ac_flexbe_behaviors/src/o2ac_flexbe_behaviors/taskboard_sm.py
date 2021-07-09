#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from o2ac_flexbe_behaviors.bearing_taskboard_sm import bearingtaskboardSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 13 2021
@author: Cristian Beltran
'''
class taskboardSM(Behavior):
	'''
	taskboard overview
	'''


	def __init__(self):
		super(taskboardSM, self).__init__()
		self.name = 'taskboard'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(bearingtaskboardSM, 'bearing taskboard')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:786 y:158, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:255 y:145
			OperatableStateMachine.add('bearing taskboard',
										self.use_behavior(bearingtaskboardSM, 'bearing taskboard'),
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
