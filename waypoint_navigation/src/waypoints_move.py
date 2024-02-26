#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from docking_control.srv import DockingControll

class MoveNode:
    def __init__(self):
        # Initialize the SimpleActionClient for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Flag to indicate if the robot is charging
        self.is_charging_flag = False

    def go_to(self, x, y, z):
        # Sends a goal to the move_base action server to navigate to the specified position
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.w = 1.0
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def home_position(self, loc):
        # Moves the robot to predefined positions based on the location index
        locations = [
            (-1.4546416903393358 -0.641329789222033 -3.1061676737127066),  # Pick Up Point
            (2.032005622785776 -0.4296098693076119 1.5367275326098822),          # First Point
            (1.940174193689601 1.2296491183212825 -3.119703896897448),   # Second Point
            (-1.2378554891366695 1.1907703806554426 1.602054918556322),  # Third Point
            (-1.2921051352450215 3.3484391575210317 -0.026924226189677943),  # Fourth Point
            (2.2067644363104617 3.4152147306981977 0.010677733754027252)      # Drop Point
        ]
        if 0 <= loc < len(locations):
            rospy.loginfo("Heading to location %d", loc)
            self.go_to(*locations[loc])
        else:
            rospy.logerr("Invalid location index: %d", loc)

    def service_call_docking(self):
        # Calls the docking service to align the robot with a docking station
        rospy.wait_for_service('docking_service')
        try:
            rospy.loginfo("Aligning Robot to Docking Station")
            server_proxy = rospy.ServiceProxy('docking_service', DockingControll)
            response = server_proxy()
            if response.value == 1:
                rospy.loginfo("Docking successful")
                self.is_charging_flag = False
            else:
                rospy.logwarn("Docking failed")
                self.is_charging_flag = True
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def loop_navigation(self):
        # Sequentially navigate through predefined positions
        for loc in range(6):
            self.home_position(loc)

    def run(self):
        try:
            rospy.loginfo("Navigation node started.")
            while not rospy.is_shutdown():
                self.loop_navigation()
                self.service_call_docking()
                # Add other operations or conditions here if needed
                rospy.sleep(1)  # Adjust sleep time as needed
        except rospy.ROSInterruptException:
            rospy.loginfo("Shutting down navigation node...")
        except Exception as e:
            rospy.logerr("An error occurred: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('navigation_node')  
    move_node = MoveNode()
    move_node.run()

