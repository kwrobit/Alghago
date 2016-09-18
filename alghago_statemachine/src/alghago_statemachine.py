#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String


class INIT(smach.State) :
    def __init__(self) :
        smach.State.__init__(self, outcomes=['user_first', 'robot_first'])
    
    def execute(self, userdata) :
        while True:
            transition = rospy.wait_for_message('/alghago_sm/transition', String)
            
            if transition.data in self._outcomes:
                return transition.data
        
        
class Thinking(smach.State) :
    def __init__(self) :
        smach.State.__init__(self, outcomes=['think_done'])
    
    def execute(self, userdata) :
        while True:
            transition = rospy.wait_for_message('/alghago_sm/transition', String)
            
            if transition.data in self._outcomes:
                return transition.data
        
        
class Waiting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['user_done', 'user_win', 'robot_win', 'draw'])
    
    def execute(self, userdata):
        while True:
            transition = rospy.wait_for_message('/alghago_sm/transition', String)
            
            if transition.data in self._outcomes:
                return transition.data
            
class Shooting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shoot_done', 'user_win', 'robot_win', 'draw'])
    
    def execute(self, userdata):
        while True:
            transition = rospy.wait_for_message('/alghago_sm/transition', String)
            
            if transition.data in self._outcomes:
                return transition.data
            
            
            
def main() :
    rospy.init_node('alghago_statemachine')
    
    sm_top = smach.StateMachine(outcomes=['USER_WIN', 'ROBOT_WIN', 'DRAW'])
    
    with sm_top :
        smach.StateMachine.add('INIT', INIT(), transitions={'user_first':'Waiting', 'robot_first':'Thinking'})
        smach.StateMachine.add('Waiting', Waiting(), transitions={'user_done':'Thinking', 'user_win':'USER_WIN', 'robot_win':'ROBOT_WIN', 'draw':'DRAW'})
        smach.StateMachine.add('Thinking', Thinking(), transitions={'think_done':'Shooting'})
        smach.StateMachine.add('Shooting', Shooting(), transitions={'shoot_done':'Waiting', 'user_win':'USER_WIN', 'robot_win':'ROBOT_WIN', 'draw':'DRAW'})
    
    # Execute SMACH plan
    sis = smach_ros.IntrospectionServer('alghago_sm', sm_top, '/SM_START')
    sis.start()
    outcome = sm_top.execute()    
    rospy.spin()
    sis.stop()
    
    rospy.signal_shutdown('fin')
    
    
if __name__ == '__main__':
    main()

