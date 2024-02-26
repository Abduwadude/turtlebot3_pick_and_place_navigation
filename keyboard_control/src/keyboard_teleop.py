#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy, rospkg

from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import sys, select, termios, tty
import math

rospack = rospkg.RosPack() #rospack is a command-line tool for retrieving information about ROS packages available on the filesystem

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

y : SAVE THE CURRENT LOCATION AS TRAJECTORY POINT
    (only works when GPS navigation is activated)

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }
          
#We are going to use the 'y key' to save coordinates to our file
is_write =  False #write status
angle = False
file_path = rospy.get_param("/outdoor_waypoint_nav/coordinates_file", "/waypoints/points.txt") #default_param = rospy.get_param('default_param', 'default_value')

def store_coordinates(x, y,Y):
    global is_write
    #construct the absolute file path
    abs_path = rospack.get_path("waypoints") + file_path
    with open(abs_path, 'a') as f:
        #lat = 
        f.write('\n'+str(x).strip() + ' ' + str(y).strip() + ' ' + str(Y).strip()) #strip() returns a copy of the string in which all chars have been stripped from the beginning and the end of the string
   # print('\n %s: %s' %(lati_point, longti_point))
    #print(abs_path)
    print('\n coordinate saved!!\n')
    #set the is_write to false
    is_write = False
def odom_CB(data):
    global angle
    if angle:
        x= data.pose.pose.position.x
        y= data.pose.pose.position.y
        qx= data.pose.pose.orientation.x
        qy= data.pose.pose.orientation.y
        qz= data.pose.pose.orientation.z
        qw= data.pose.pose.orientation.w
        (R,P,Y)= euler_from_quaternion((qx,qy,qz,qw))
        print(math.degrees(Y)) 
        angle = False
    if(is_write):
    
    
        x= data.pose.pose.position.x
        y= data.pose.pose.position.y
        qx= data.pose.pose.orientation.x
        qy= data.pose.pose.orientation.y
        qz= data.pose.pose.orientation.z
        qw= data.pose.pose.orientation.w
        (R,P,Y)= euler_from_quaternion((qx,qy,qz,qw))
        store_coordinates(x, y, Y)
#     sub_amcl = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, amcl_CB)
# def amcl_CB(data): #the function that is passed as the input argument to another function is called a callback function in Python.
   
  
    
        


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = .26
turn = 1

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('custom_keyboard_teleop') #One of the first calls you will likely execute in a rospy program is the call to rospy.init_node(), which initializes the ROS node for the process. You can only have one node in a rospy process, so you can only call rospy.init_node() once. 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5) #/joy_teleop/cmd_vel
    #queue_size is the size of the outgoing message queue used for asynchronous publishing.
    
    #receiving data from the satellite device
    sub_odom = rospy.Subscriber('/odom',Odometry, odom_CB)

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print (msg)
        print (vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                count = 0
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                count = 0

                print (vels(speed,turn))
                if (status == 14):
                    print (msg)
                status = (status + 1) % 15
            elif key == 'y':
                print('pressed')
                is_write = True
               
            elif key == ' ' or key == 'k' :
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            elif key == 'a':
                angle = True

            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break
        
            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min( target_speed, control_speed + 0.02 )
            elif target_speed < control_speed:
                control_speed = max( target_speed, control_speed - 0.02 )
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min( target_turn, control_turn + 0.1 )
            elif target_turn < control_turn:
                control_turn = max( target_turn, control_turn - 0.1 )
            else:
                control_turn = target_turn

            twist = Twist()
            twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
            pub.publish(twist)

            #print("loop: {0}".format(count))
            #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
            #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print (e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
