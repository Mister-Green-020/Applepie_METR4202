#!/usr/bin/env python3
import constants
import roslib
import rospy 
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import numpy as np
from constants import *

class Setup(smach.State):
    """
    Setup state aimed at checking motors work and initialising camera detection
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup'])
        self.position_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.camera_pub = rospy.Publisher('/ximea_ros/show_rgb', Bool, queue_size=10)
        self.setup_state = setup_pose


    def execute(self, userdata):
        rospy.loginfo('Executing state setup')
        show_rgb = Bool()
        show_rgb.data = True

        self.camera_pub(show_rgb)
        self.position_pub.publish(self.setup_state)

        return 'setup'
        

class InitialState(smach.State):
    """
    Initial state sets the arm to a specified initial configuration.
    Publishes to the gripper and a new position.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialised'])
        self.new_position = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.gripper_open = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.initial_config = init_pose

    def execute(self, userdata):
        rospy.loginfo('Executing state Initial')
        self.gripper_open.publish(True)
        self.new_position.publish(self.initial_config)

        return 'initialised'

class FindBlock(smach.State):
    def __init__(self):
        """
        While loop runs until it receives message of a block position, including block type
        http://wiki.ros.org/smach/Tutorials/User%20Data
        """
        smach.State.__init__(self, outcomes=['position_found'], output_keys=['block_transform'])
        self.positions = rospy.Subscriber('/block_positions', Pose, self.transform)
        
        self.block_found = False
        self.transform = Pose()

    def transform(self, tf):
        self.transform = tf
        self.block_found = True
        
    
    def execute(self, userdata):
        rospy.loginfo('Executing state FindBlock')
        
        while not self.block_found :
            pass

        rospy.loginfo('Position found')

        userdata.block_transform = self.transform
        self.block_found = False
        return 'position_found'


class MoveToBlock(smach.State):
    """
    Camera provides next location of block, this will subscribe to the value and publish it to joints
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['positioned'], input_keys=['block_transform'])
        self.new_position = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state Move')
        self.new_position.publish(userdata.block_transform)
        return 'positioned'
        

class GrabBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed'])
        self.gripper = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state GrabBlock')
        self.gripper.publish(False)
        return 'grabbed'

class MoveToIdentifyPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['identified'])
        self.checking_pose = id_pose
        self.pos_pub = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing identify position')
        self.new_position.publish(self.checking_pose)
        return 'in_identify_position'

class IdentifyBlock(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['identified'])
        self.colour_sub = rospy.Subscriber('/block_colour', String, queue_size=10)
        self.colour = 0

    def execute(self, userdata):

        while (self.colour == 0) :
            rospy.loginfo('Waiting for colour')

        # Pass colour to next state
        return 'identified'


class MoveToDrop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_positioned'])
        self.sub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.zone_1_blocks = 0
        self.zone_2_blocks = 0
        self.zone_3_blocks = 0
        self.zone_4_blocks = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToDrop')

        colour = "red"
        if colour == red_zone.colour :
            self.pose_pub.publish(red_zone.pose)
            self.zone_1_blocks += 1
        elif colour == blue_zone.colour :
            self.pose_pub.publish(blue_zone.pose)
            self.zone_2_blocks += 1
        if colour == green_zone.colour :
            self.pose_pub.publish(green_zone.pose)
            self.zone_3_blocks += 1
        elif colour == yellow_zone.colour :
            self.pose_pub.publish(yellow_zone.pose)
            self.zone_4_blocks += 1
        
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
    sm = smach.StateMachine(outcomes=['released'])

    with sm:
        smach.StateMachine.add('Setup', Setup(), 
                transitions={'setup' : 'InitialState'})

        smach.StateMachine.add('InitialState', InitialState(), 
                        transitions={'initialised' : 'FindBlock'})

        smach.StateMachine.add('FindBlock', FindBlock(), 
                        transitions={'position_found':'MoveToBlock'},
                        remapping={'block_transform':'sm_block_transform'})

        smach.StateMachine.add('MoveToBlock', MoveToBlock(), 
                        transitions={'positioned':'GrabBlock'},
                        remapping={'block_transform':'sm_block_transform'})

        smach.StateMachine.add('GrabBlock', GrabBlock(), 
                        transitions={'grabbed':'MoveToIdentifyPosition'})
        
        smach.StateMachine.add('MoveToIdentifyPosition', MoveToIdentifyPosition(), 
                        transitions={'in_identify_position':'IdentifyBlock'})

        smach.StateMachine.add('IdentifyBlock', MoveToIdentifyPosition(), 
                        transitions={'identified':'MoveToDrop'})

        smach.StateMachine.add('MoveToDrop', ReleaseBlock(), 
                        transitions={'drop_positioned':'ReleaseBlock'})

        smach.StateMachine.add('ReleaseBlock', InitialState(), 
                        transitions={'released':'InitialState'})
        
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()



if __name__ == '__main__':
    try :
        main()
    except (rospy.ROSInterruptException) :
        rospy.loginfo('Ended')