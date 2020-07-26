#!/usr/bin/env python2

import rospy, math
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from roslaunch.parent import ROSLaunchParent
from std_msgs.msg import Int32


global x_cheese, y_cheese, x_cat, y_cat, x_mouse, y_mouse, game_state



def cat_odom_callback(odom):
    global x_mouse, y_mouse, x_cat, y_cat, game_state
    x_cat = odom.pose.pose.position.x
    y_cat = odom.pose.pose.position.y
    dist = math.sqrt((x_mouse - x_cat)**2 + (y_mouse - y_cat)**2)
    if dist < 0.2 and game_state == 0:
        game_state = 2
    if game_state == 2:
    		rospy.loginfo("mouse was caught. cat wins")
    		print("game over. cat wins")


def mouse_odom_callback(odom):
    global x_mouse, y_mouse, x_cat, y_cat, game_state
    x_mouse = odom.pose.pose.position.x
    y_mouse = odom.pose.pose.position.y
    dist = math.sqrt((x_mouse - x_cheese)**2 + (y_mouse - y_cheese)**2)
    rospy.loginfo("new distance to cheese: " + str(dist))
    print("new mouse distance to cheese: " + str(dist))
    if dist < 0.2 and game_state == 0:
        game_state = 1 
    if (game_state == 1):
    		rospy.loginfo("cheese was caught. mouse wins")
    		print("game over. mouse wins")



if __name__ == '__main__':
    global x_cheese, y_cheese, x_cat, y_cat, x_mouse, y_mouse, game_state
    
    game_state = Int32()
    game_state = 0
    
    x_cheese = rospy.get_param('cheese_pos_x')
    y_cheese = rospy.get_param('cheese_pos_y')

    x_cat = rospy.get_param('cat_start_pos_x')
    y_cat = rospy.get_param('cat_start_pos_y')

    x_mouse = rospy.get_param('mouse_start_pos_x')
    y_mouse = rospy.get_param('mouse_start_pos_y')


    rospy.init_node('refree', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pub = rospy.Publisher("game_state", Int32, queue_size=10)
    rospy.Subscriber("cat/odom", Odometry, cat_odom_callback)
    rospy.Subscriber("mouse/odom", Odometry, mouse_odom_callback)

    while not rospy.is_shutdown():
        pub.publish(game_state)
        rate.sleep()
