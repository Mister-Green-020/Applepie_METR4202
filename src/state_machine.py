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
        rospy.sleep(sleep_s)

        return 'setup'
        

class InitialState(smach.State):
    """
    Initial state sets the waiting position for the arm to grab blocks
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialised'])
        self.pose_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.gripper_pub = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.initial_config = init_pose
        self.open_gripper = Bool(
            data=True
        )

    def execute(self, userdata):

        """
        Sets robot to initial state
        """

        rospy.loginfo('Executing state Initial')
        self.gripper_pub.publish(self.open_gripper)
        self.pose_pub.publish(self.initial_config)
        rospy.sleep(1)

        return 'initialised'

class FindBlock(smach.State):
    def __init__(self):

        """

        """
       
        smach.State.__init__(self, outcomes=['position_found'], output_keys=['block_transform'])
        self.positions = rospy.Subscriber('/block_positions', Pose, self.transform)
        
        self.block_found = False
        self.pose = Pose()

    def transform(self, new_pose: Pose):

        """
        Identifies the new pose if a block is found
        """

        self.pose = new_pose
        self.block_found = True
        
    
    def execute(self, userdata):

        """
        While loop runs until it has located a block position and passes the transform onto the next state
        """

        rospy.loginfo('Executing state FindBlock')
        rospy.sleep(1)
        while not self.block_found :
            pass

        rospy.loginfo('Position found')

        userdata.block_transform = self.pose
        self.block_found = False
        return 'position_found'


class MoveToBlock(smach.State):
    """
    Moves the robot arm to the desired location from the waiting stage via userdata
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['positioned'], input_keys=['block_transform'])
        self.new_position = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):
        """
        Robot arm moves to new position from transform information
        """
        rospy.loginfo('Executing state Move')
        self.new_position.publish(userdata.block_transform)
        rospy.sleep(sleep_s)
        return 'positioned'
        

class GrabBlock(smach.State):
    """
    Command state to close the gripper
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['grabbed'])
        self.pub = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.grab = Bool(
            data=False
        )


    def execute(self, userdata):

        """
        Gripper closes and grabs the block
        """

        rospy.loginfo('Executing state GrabBlock')
        self.pub.publish(self.grab)
        # rospy.sleep(sleep_s)
        return 'grabbed'

class MoveToIdentifyPosition(smach.State):
    """
    State to move the block to a position in which the colour can be easily determined
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['in_identify_position'])
        self.checking_pose = id_pose
        self.pos_pub = rospy.Publisher('/new_position', Pose, queue_size=10)


    def execute(self, userdata):

        """
        Robot moves the block to specified checking position, holding for 1 second to allow colour to be checked
        """

        rospy.loginfo('Executing identify position')
        self.pos_pub.publish(self.checking_pose)
        rospy.sleep(sleep_s)
        return 'in_identify_position'


class IdentifyBlock(smach.State):

    """
    Identifies whether the robot is holding a block and what colour the block is
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['identified', 'no_block'], output_keys=['block_colour'])
        self.colour_sub = rospy.Subscriber('/block_colour', String, self.callback)
        self.colour = "'none'"
        self.time = 0
    

    def execute(self, userdata):

        """
        Robot to do nothing if colour is not identified.
        If colour is identified, data is passed on to the next state.
        """

        self.time = rospy.get_time()

        while (self.colour == "'none'") :
            if ((rospy.get_time() - self.time) > wait_time) :
                return 'no_block'
            pass

        # Pass colour to next state
        userdata.block_colour = self.colour
        self.colour = "none"
        return 'identified'
    
    def callback(self, colour: String) :
        self.colour = colour.data



class MoveToDrop(smach.State):

    """
    Robot moves the block to the zone corresponding to the colour of the block
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_positioned'], input_keys=['block_colour'])
        self.pose_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.zone_1_blocks = 0
        self.zone_2_blocks = 0
        self.zone_3_blocks = 0
        self.zone_4_blocks = 0

    def execute(self, userdata):

        """
        The only conditions assessed are whether the block is one of the 4 specified colours.
        The condition to determine whether a block is present was run within the IdentifyBloch class avove.
        """

        rospy.loginfo('Executing state MoveToDrop')

        colour = userdata.block_colour
        if colour == red_zone.colour :
            self.pose_pub.publish(red_zone.pose)
            self.zone_1_blocks += 1
        elif colour == blue_zone.colour :
            self.pose_pub.publish(blue_zone.pose)
            self.zone_2_blocks += 1
        elif colour == green_zone.colour :
            self.pose_pub.publish(green_zone.pose)
            self.zone_3_blocks += 1
        elif colour == yellow_zone.colour :
            self.pose_pub.publish(yellow_zone.pose)
            self.zone_4_blocks += 1

        rospy.sleep(sleep_s)
        return 'drop_positioned'


class ReleaseBlock(smach.State):
    
    """
    Command state to close the gripper
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['released'])
        self.gripper = rospy.Publisher('/desired_gripper_position', Bool, queue_size=10)
        self.release = Bool(
            data=True
        )

    def execute(self, userdata):

        """
        Gripper opens and releases the block
        """

        rospy.loginfo('Executing state Release')
        self.gripper.publish(self.release)
        return 'released'


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
                        transitions={'identified':'MoveToDrop', 'no_block' : 'InitialState'})

        smach.StateMachine.add('MoveToDrop', MoveToDrop(), 
                        transitions={'drop_positioned':'ReleaseBlock'})

        smach.StateMachine.add('ReleaseBlock', ReleaseBlock(), 
                        transitions={'released':'InitialState'})
        
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()



if __name__ == '__main__':
    try :
        main()
    except (rospy.ROSInterruptException) :
        rospy.loginfo('Ended')