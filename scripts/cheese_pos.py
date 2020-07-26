#!/usr/bin/env python2

import rospy, sys
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

def cheese_pose(x_pos, y_pos):

    pub = rospy.Publisher('cheese_pos', Pose, queue_size=10)
    rospy.init_node('cheese_pos', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pose = Pose()
    pose.position.x = x_pos  
    pose.position.y = y_pos  

    while not rospy.is_shutdown():

        pub.publish(pose)
        rate.sleep()

def set_model_state(x_pos, y_pos):
    state_msg = ModelState()
    state_msg.model_name = 'cheese1'
    state_msg.pose.position.x = x_pos  
    state_msg.pose.position.y = y_pos 

    rospy.wait_for_service('/gazebo/set_model_state')
    rospy.sleep(1)
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':

    x = rospy.get_param('cheese_pos_x')
    y = rospy.get_param('cheese_pos_y')
    print(x)
    #args = rospy.myargv(argv=sys.argv)

    #if len(sys.argv) < 4 :
    #    print("TEST TEST TEST")  #sys.argv[1], sys.argv[2])
    set_model_state(x, y)

    try:
        cheese_pose(x, y)
    except rospy.ROSInterruptException:
        pass
