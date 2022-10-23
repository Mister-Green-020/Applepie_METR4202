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
    
    def __init__(self):
        """
        Setup state occurs once on launch, to verify motors function and ensure RGB colour is on.
        """  
        smach.State.__init__(self, outcomes=['setup'])
        self.position_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.camera_pub = rospy.Publisher('/ximea_ros/show_rgb', Bool, queue_size=10)
        self.setup_state = setup_pose


    def execute(self, userdata):
        rospy.loginfo('Executing state setup')
        show_rgb = Bool(
            data=True
        )

        self.camera_pub.publish(show_rgb)
        self.position_pub.publish(self.setup_state)
        rospy.sleep(2)

        return 'setup'
        

class InitialState(smach.State):
    
    def __init__(self):
        """
        Initial state sets the waiting position for the arm to grab blocks
        """
        smach.State.__init__(self, outcomes=['initialised'])
        self.pose_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.gripper_pub = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.initial_config = init_pose
        self.open_gripper = Bool(
            data=True
        )

    def execute(self, userdata):
        rospy.loginfo('Executing state Initial')
        self.gripper_pub.publish(self.open_gripper)
        self.pose_pub.publish(self.initial_config)
        rospy.sleep(1)

        return 'initialised'

class FindBlock(smach.State):
    def __init__(self):
        """
        Waiting phase of the robot, which will wait indefinitely until
        a valid block position has been found.
        """
        smach.State.__init__(self, outcomes=['position_found'], output_keys=['block_transform'])
        self.positions = rospy.Subscriber('/block_positions', Pose, self.transform)
        
        self.block_found = False
        self.pose = Pose()

    def transform(self, new_pose: Pose):
        """
        Callback function which will update variables if there is a valid
        transform
        """
        self.pose = new_pose
        self.block_found = True
        
    
    def execute(self, userdata):
        rospy.loginfo('Executing state FindBlock')
        rospy.sleep(sleep_s)
        while not self.block_found :
            pass

        rospy.loginfo('Position found')

        userdata.block_transform = self.pose
        self.pose = init_pose
        self.block_found = False
        return 'position_found'


class MoveToBlock(smach.State):
    
    def __init__(self):
        """
        Moves the robot arm to the desired location from the waiting stage via userdata
        """
        smach.State.__init__(self, outcomes=['positioned'], input_keys=['block_transform'])
        self.new_position = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing state Move')
        self.new_position.publish(userdata.block_transform)
        rospy.sleep(0.7)
        return 'positioned'
        

class GrabBlock(smach.State):
    
    def __init__(self):
        """
        Command state to close the gripper
        """
        smach.State.__init__(self, outcomes=['grabbed'])
        self.pub = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.grab = Bool(
            data=False
        )

    def execute(self, userdata):
        rospy.loginfo('Executing state GrabBlock')
        self.pub.publish(self.grab)
        # rospy.sleep(sleep_s)
        return 'grabbed'

class MoveToIdentifyPosition(smach.State):
    
    def __init__(self):
        """
        State to move the block to a position in which the colour can be easily determined
        """
        smach.State.__init__(self, outcomes=['in_identify_position'])
        self.checking_pose = id_pose
        self.pos_pub = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):
        rospy.loginfo('Executing identify position')
        self.pos_pub.publish(self.checking_pose)
        rospy.sleep(sleep_s)
        return 'in_identify_position'


class IdentifyBlock(smach.State):
    def __init__(self):
        """
        Identifies whether the robot is holding a block and what colour the block is
        """
        smach.State.__init__(self, outcomes=['identified', 'no_block'], output_keys=['block_colour'])
        self.colour_sub = rospy.Subscriber('/block_colour', String, self.callback)
        self.colour = "'none'"
        self.time = 0
    

    def execute(self, userdata):
        self.time = rospy.get_time()

        while (self.colour == "'none'") :
            if ((rospy.get_time() - self.time) > wait_time) :
                return 'no_block'
            pass

        # Pass colour to next state
        userdata.block_colour = self.colour
        self.colour = "'none'"
        return 'identified'
    
    def callback(self, colour: String) :
        self.colour = colour.data


class GoToThrowPos(smach.State):
    def __init__(self):
        """
        State to move to the correct pre-throw position to generate enough 
        velocity upon movement and aim the correct way.
        """
        # State takes in the colour and passes it on as it requires 2 positions
        # to determine drop location
        smach.State.__init__(self, outcomes=['in_throw_pos'], 
            input_keys=['block_colour'], output_keys=['colour'])
        self.pose_pub = rospy.Publisher('/new_position', Pose, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToDrop')
        colour = userdata.block_colour
        # Helpful log just to verify colours are calibrated correctly
        rospy.loginfo(colour)
        # No use for a default case as it cannot enter this state without a colour
        if colour == red_zone_throw.colour :
            self.pose_pub.publish(red_zone_throw.pose)
        elif colour == blue_zone_throw.colour :
            self.pose_pub.publish(blue_zone_throw.pose)
        elif colour == green_zone_throw.colour :
            self.pose_pub.publish(green_zone_throw.pose)
        elif colour == yellow_zone_throw.colour :
            self.pose_pub.publish(yellow_zone_throw.pose)

        rospy.sleep(throw_wait)
        userdata.colour = colour
        return 'in_throw_pos'

class SafetyDrop(smach.State):
    
    def __init__(self):
        """
        Safety state executes if colour could not be identified, intended to
        act as a precaution in case the colour is not visible - so it will drop
        the block back on the conveyor. 
        """
        smach.State.__init__(self, outcomes=['can_safely_drop'])
        self.safety_pose = safety_pose
        self.grip_pub = rospy.Publisher('/desired_gripper_state', Bool, queue_size=10)
        self.pos_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.release = Bool(
            data=True
        )


    def execute(self, userdata):

        """
        Robot moves the block to specified checking position, holding for 1 second to allow colour to be checked
        """

        rospy.loginfo('Executing identify position')
        self.pos_pub.publish(self.safety_pose)
        rospy.sleep(sleep_s)
        self.grip_pub.publish(self.release)
        return 'can_safely_drop'

class ThrowBlock(smach.State):
    def __init__(self):
        """
        State to execute a throw of the block, achieved by moving the arm
        and releasing the block in the same movement to generate a 
        sufficient velocity.
        """
        smach.State.__init__(self, outcomes=['bye_bye_block'], input_keys=['colour'])
        self.pose_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.gripper = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.release = Bool(
            data=True
        )

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToDrop')
        colour = userdata.colour
        # Get the location of the original drop zones to aim for
        if colour == red_zone.colour :
            self.pose_pub.publish(red_zone.pose)
        elif colour == blue_zone.colour :
            self.pose_pub.publish(blue_zone.pose)
        elif colour == green_zone.colour :
            self.pose_pub.publish(green_zone.pose)
        elif colour == yellow_zone.colour :
            self.pose_pub.publish(yellow_zone.pose)

        # Sleep calibration to wait briefly before releasing
        rospy.sleep(throw_release)
        self.gripper.publish(self.release)
        return 'bye_bye_block'



def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=[''])
    # sm = smach.StateMachine(outcomes=['released'])

    with sm:
        smach.StateMachine.add('Setup', Setup(), 
                transitions={'setup' : 'InitialState'})

        smach.StateMachine.add('InitialState', InitialState(), 
                        transitions={'initialised' : 'FindBlock'})

        smach.StateMachine.add('FindBlock', FindBlock(), 
                        transitions={'position_found':'MoveToBlock'})

        smach.StateMachine.add('MoveToBlock', MoveToBlock(), 
                        transitions={'positioned':'GrabBlock'})

        smach.StateMachine.add('GrabBlock', GrabBlock(), 
                        transitions={'grabbed':'MoveToIdentifyPosition'})
        
        smach.StateMachine.add('MoveToIdentifyPosition', MoveToIdentifyPosition(), 
                        transitions={'in_identify_position':'IdentifyBlock'})

        smach.StateMachine.add('IdentifyBlock', IdentifyBlock(), 
                        transitions={'identified':'GoToThrowPos', 'no_block' : 'SafetyDrop'})
        
        smach.StateMachine.add('SafetyDrop', SafetyDrop(), 
                        transitions={'can_safely_drop':'InitialState'})

        smach.StateMachine.add('GoToThrowPos', GoToThrowPos(), 
                        transitions={'in_throw_pos' : 'ThrowBlock'})

        smach.StateMachine.add('ThrowBlock', ThrowBlock(), 
                        transitions={'bye_bye_block' : 'InitialState'})
        

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()



if __name__ == '__main__':
    try :
        main()
    except (rospy.ROSInterruptException) :
        rospy.loginfo('Ended')