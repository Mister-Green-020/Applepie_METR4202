#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import numpy as np

class InitialState(smach.State):
    """
    Initial state sets the arm to a specified initial configuration.
    Publishes to the gripper and a new position.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialised'])
        self.new_position = rospy.Publisher('/desired_joint_states', TransformStamped, queue_size=10)
        self.gripper_open = rospy.Publisher('/desired_gripper_position', TransformStamped, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Executing state Initial')
        self.gripper_open.publish(True)

        return 'initialised'


class MoveToBlock(smach.State):
    """
    
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['positioned'])
        self.block_position = rospy.Subscriber('/block_positions', TransformStamped, self.valid_transform)
        self.new_position = rospy.Publisher('/desired_joint_states', TransformStamped, queue_size=10)

    def valid_transform(self, data) :
        rospy.loginfo('')


    def execute(self, userdata):
        rospy.loginfo('Executing state Move')
        return 'positioned'
        

class GrabBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed'])
        self.gripper = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state GrabBlock')
        self.gripper.publish(False)
        return 'grabbed'


class MoveToDrop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_positioned'])
        self.new_position = rospy.Publisher('/desired_joint_states', TransformStamped, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToDrop')
        
        return 'drop_positioned'


class ReleaseBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['released'])
        self.gripper = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state Release')
        self.gripper.publish(True)
        return 'released'




def main():
    rospy.init_node('state_machine_main')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    with sm:
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()