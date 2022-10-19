#!/usr/bin/env python3
import constants
import roslib
import rospy 
import smach
import smach_ros
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import numpy as np

class Setup(smach.State):
    """
    Setup state aimed at checking motors work and initialising camera detection
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['setup'])
        self.position_pub = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.camera_pub = rospy.Publisher('/ximea_ros/show_rgb', Bool, queue_size=10)
        self.setup_state = Pose()

        # Need someone to set these to fully upright (all joints straight)
        self.setup_state.position.x = 0
        self.setup_state.position.y = 0
        self.setup_state.position.z = constants.L1 + constants.L2 + constants.L3 + constants.L4


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
        self.initial_config = Pose()
        
        # Need someone to set these to L joint ->
        #    First joint straight
        #    Second joint at 90 deg
        #    third and fourth straight
        self.setup_state.position.x = 0
        self.setup_state.position.y = constants.L3 + constants.L4
        self.setup_state.position.z = constants.L1 + constants.L2


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
            rospy.sleep(1)
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


class MoveToDrop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['drop_positioned'])
        self.new_position = rospy.Publisher('/new_position', Pose, queue_size=10)
        self.zone_1_blocks = 0
        self.zone_2_blocks = 0
        self.zone_3_blocks = 0
        self.zone_4_blocks = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToDrop')
        move_to = Pose()

        # Some way to associate colours with enum
        zone = 1

        if zone == 1:
            move_to.point.x = 0
            move_to.point.y = 0
            move_to.point.z = 0
            self.zone_1_blocks += 1

        elif zone == 2:
            move_to.point.x = 0
            move_to.point.y = 0
            move_to.point.z = 0
            self.zone_2_blocks += 1

        elif zone == 3 :
            move_to.point.x = 0
            move_to.point.y = 0
            move_to.point.z = 0
            self.zone_3_blocks += 1

        else :
            move_to.point.x = 0
            move_to.point.y = 0
            move_to.point.z = 0
            self.zone_4_blocks += 1

        self.new_position.publish(move_to)
        
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
                        transitions={'grabbed':'MoveToBlock'})

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