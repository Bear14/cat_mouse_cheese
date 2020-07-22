#!/usr/bin/env python

import rospy
import numpy as np
import math as m
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

CHEESE_INTEREST_FACTOR = 0.5
CONSTANT_CAT_SPEED = 0.22
MAX_TURNING_FACTOR = 1


class Cat:
    def __init__(self):
        self.x_cheese = rospy.get_param('cheese_pos_x')
        self.y_cheese = rospy.get_param('cheese_pos_y')
        self.x_mouse = None
        self.y_mouse = None
        self.x_cat = None
        self.y_cat = None
        self.attraction_point_x = None
        self.attraction_point_y = None
        self.sensor_angles = None
        self.sensor_ranges = None

        self.rho = 1
        self.alpha = 1
        self.phi_cat = 1

        rospy.Subscriber("cat/odom", Odometry, self.cat_odom_callback)
        rospy.Subscriber("mouse/odom", Odometry, self.mouse_odom_callback)
        rospy.Subscriber("cat/scan", LaserScan, self.laser_callback)

        self.cat_publisher = rospy.Publisher("cat/cmd_vel", Twist, queue_size=10)
        rospy.spin()

    def cat_odom_callback(self, odom):
        self.x_cat = odom.pose.pose.position.x
        self.y_cat = odom.pose.pose.position.y

        quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w]
        euler = euler_from_quaternion(quaternion)
        self.phi_cat = euler[2]

        self.set_state()

    def mouse_odom_callback(self, odom):
        self.x_mouse = odom.pose.pose.position.x
        self.y_mouse = odom.pose.pose.position.y

        quaternion = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
                       odom.pose.pose.orientation.w]
        euler = euler_from_quaternion(quaternion)
        self.phi_mouse = euler[2]
        # # self.update_polar()
        # # self.homing()
        self.angular_mouse = odom.twist.twist.angular.z
        self.linear_mouse = odom.twist.twist.linear.x
        self.set_state()

    def laser_callback(self, current_laser_scan):
        self.sensor_angles = np.arange(current_laser_scan.angle_min,
                                       current_laser_scan.angle_max + current_laser_scan.angle_increment,
                                       current_laser_scan.angle_increment)
        self.sensor_ranges = np.array(current_laser_scan.ranges)

    def calc_attraction_point(self):
        # calc Vector
        r =  1 #self.linear_mouse
        phi = self.phi_mouse
        x = m.cos(phi) * r
        y = m.sin(phi) * r

        self.attraction_point_x = x + self.x_mouse
        self.attraction_point_y = y + self.y_mouse
        #print(x,y)

        if self.x_mouse and self.y_mouse:
            self.attraction_point_x = (self.x_mouse + self.x_cheese) * 0.5
            self.attraction_point_y = (self.y_mouse + self.y_cheese) * 0.5

    def update_polar(self, goal_x, goal_y):
        # make sure cat_odom_callback is at least called once
        while self.x_cat is None and self.y_cat is None:
            print("Wait for cat_odom_callback.")

        delta_x = goal_x - self.x_cat
        delta_y = goal_y - self.y_cat

        self.roh = m.sqrt(delta_x ** 2 + delta_y ** 2)  # Distanz zum ziel
        #self.alpha = m.atan2(delta_y, delta_x) - self.phi_cat # Winkel zum ziel #Verhindert Drehung?

        buff = m.atan2(delta_y,delta_x)

        if(delta_x < 0 and delta_y > 0):
            buff = buff + m.pi
        if(delta_x < 0 and delta_y < 0):
            buff = buff + m.pi

        self.alpha = buff - self.phi_cat

        print(self.alpha, "= Tan(", delta_y,"|", delta_x,"-",self.phi_cat )

    def calculate_force(self):

        force = np.zeros(2)

        while self.sensor_ranges is None and self.sensor_angles is None:
            print("Wait for laser_callback.")

        for i in range(len(self.sensor_ranges)):
            if (self.sensor_ranges[i] < 0.8):
                force[1] += -m.sin(self.sensor_angles[i]) * (0.8 - self.sensor_ranges[i])

        force[1] /= 24
        if ((force[1] > -0.1 and force[1] < 0.1)):
            force[1] *= 2
        elif (force[1] < -0.8):
            force[1] = -0.8
        elif (force[1] > 0.8):
            force[1] = 0.8

        return force

    def homing(self):
        k_rho_0 = 0.1
        k_alpha_0 = 0.3

        # force = self.calculate_force(self.sensor_angles, self.sensor_ranges)
        #
        # if np.abs(self.alpha) < 0.3:       #winkel klein
        #     if np.abs(self.rho) < 0.1:          #mehr drehen wenn nah dran
        #         # k_rho_0 = 1 * k_rho_0
        #         k_alpha_0 = 1 * k_alpha_0
        #     else:
        #         # k_rho_0 = 1 * k_rho_0
        #         k_alpha_0 = 2 * k_alpha_0
        # else:                               # winkel gros fahren wir langsamer
        #     if np.abs(self.rho) < 0.1:
        #         # k_rho_0 = 2 * k_rho_0
        #         k_alpha_0 = 2 * k_alpha_0 # wenn nah dran dann mehr drehen als wenn weit weg
        #     else:
        #         # k_rho_0 = 0 * k_rho_0
        #         k_alpha_0 = 4 * k_alpha_0 # wenn weit weg dann mehr drehen als wenn nah

        out = Twist()
        out.linear.x = CONSTANT_CAT_SPEED  # k_rho_0 * self.rho # + force[0]

        force = self.calculate_force()
        #print(force)

        out.angular.z = k_alpha_0 * self.alpha + force[1]
        # if out.angular.z > 0.8:
        #     out.angular.z = 0.8
        # if out.angular.z < -0.8:
        #     out.angular.z = -0.8
        self.cat_publisher.publish(out)

    def set_state(self):
        dist_mc = m.sqrt((self.x_mouse - self.x_cheese) ** 2 + (self.y_mouse - self.y_cheese) ** 2)
        dist_cc = m.sqrt((self.x_cat - self.x_cheese) ** 2 + (self.y_cat - self.y_cheese) ** 2)

        # if dist_cc < dist_mc:  # Cat naeher an chees => Mous als Ziel
        #     self.update_polar(self.x_mouse, self.y_mouse)
        #     print("Ziel Mous")
        # else:  # Mous naeher an chees => Attraktionpoint als Ziel
        #     self.calc_attraction_point()
        #     self.update_polar(self.attraction_point_x, self.attraction_point_y)
        #     print("Ziel Kaese")
        self.calc_attraction_point()
        self.update_polar(self.attraction_point_x, self.attraction_point_y)
        self.homing()


if __name__ == '__main__':
    rospy.init_node('cat', anonymous=True)
    try:
        node = Cat()
    except rospy.ROSInterruptException:
        pass
