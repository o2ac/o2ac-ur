#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from o2ac_flexbe_states.insertion import InsertActionState
from o2ac_flexbe_states.orient import OrientActionState
from o2ac_flexbe_states.pick import PickActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on 1626679188570
@author: Cristian Beltran
'''
class TaskSkeletontoFlexbeSM(Behavior):
    '''
    test
    '''


    def __init__(self):
        super(TaskSkeletontoFlexbeSM, self).__init__()
        self.name = 'Task Skeleton to Flexbe'

        # parameters of this behavior

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]
		
		# [/MANUAL_INIT]

        # Behavior comments:



    def create(self):
        a_bot = "a_bot"
        b_bot = "b_bot"
        taskboard = "taskboard"
        assembly = "assembly"
        shaft = "shaft"
        end_cap = "end_cap"
        # x:30 y:365, x:1230 y:540
        _state_machine = OperatableStateMachine(outcomes=['failed', 'finished'])

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('pick a_bot end_cap',
                                        PickActionState(robot_name=a_bot, object_name=end_cap),
                                        transitions={'success': 'orient a_bot assembly end_cap', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

            # x:1030 y:490
            OperatableStateMachine.add('insert a_bot assembly shaft',
                                        InsertActionState(robot_name=a_bot, task_name=assembly, object_name=shaft),
                                        transitions={'success': 'finished', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

            # x:230 y:130
            OperatableStateMachine.add('orient a_bot assembly end_cap',
                                        OrientActionState(robot_name=a_bot, task_name=assembly, object_name=end_cap),
                                        transitions={'success': 'pick b_bot shaft', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

            # x:630 y:310
            OperatableStateMachine.add('orient b_bot assembly shaft',
                                        OrientActionState(robot_name=b_bot, task_name=assembly, object_name=shaft),
                                        transitions={'success': 'insert a_bot assembly end_cap', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

            # x:430 y:220
            OperatableStateMachine.add('pick b_bot shaft',
                                        PickActionState(robot_name=b_bot, object_name=shaft),
                                        transitions={'success': 'orient b_bot assembly shaft', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})

            # x:830 y:400
            OperatableStateMachine.add('insert a_bot assembly end_cap',
                                        InsertActionState(robot_name=a_bot, task_name=assembly, object_name=end_cap),
                                        transitions={'success': 'insert a_bot assembly shaft', 'error': 'failed'},
                                        autonomy={'success': Autonomy.Off, 'error': Autonomy.Off})


        return _state_machine


    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
