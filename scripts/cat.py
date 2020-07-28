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
GOAL_RADIUS = 0.7
HUNT_RADIUS = 0.862
LET_GO_RADIUS = 3
HYSTERESIS = 0.2

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

        self.state = "cheese"

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

    def distance(self, _x, _y, _x2, _y2):
        return m.sqrt((_x - _x2) ** 2 + (_y - _y2) ** 2)

    def huntingFactor(self, x):
        return 2 / x - 2 / (x ** 3) + 0.8

    def calc_preditction_mouse(self):
        # calc Vector
        distance_mouse_cat = self.distance(self.x_mouse, self.y_mouse, self.x_cat, self.y_cat)

        r = 0.18 * self.huntingFactor(distance_mouse_cat)
        if r < 0:
            r = 0
        # print(r)
        phi = self.phi_mouse + self.angular_mouse
        x = m.cos(phi) * r
        y = m.sin(phi) * r

        self.attraction_point_x = x + self.x_mouse
        self.attraction_point_y = y + self.y_mouse
        # print(x,y)

        # if self.x_mouse and self.y_mouse:
        #    self.attraction_point_x = (self.x_mouse + self.x_cheese) * 0.5
        #    self.attraction_point_y = (self.y_mouse + self.y_cheese) * 0.5

    def calc_mid_point(self):
        self.attraction_point_x = (self.x_mouse + self.x_cheese) * 0.5
        self.attraction_point_y = (self.y_mouse + self.y_cheese) * 0.5

    def update_polar(self, goal_x, goal_y):
        # make sure cat_odom_callback is at least called once
        while self.x_cat is None and self.y_cat is None:
            print("Wait for cat_odom_callback.")

        delta_x = goal_x - self.x_cat
        delta_y = goal_y - self.y_cat

        self.roh = m.sqrt(delta_x ** 2 + delta_y ** 2)  # Distanz zum ziel
        self.alpha = m.atan2(delta_y, delta_x) - self.phi_cat  # Winkel zum ziel #Verhindert Drehung?
        if (self.alpha > m.pi):
            self.alpha = m.pi - self.alpha
        elif (self.alpha < -m.pi):
            self.alpha = -m.pi - self.alpha

    def calculate_force(self):
        # force Manipulatoren
        rel_phi = 80  # ganzzahlig und positiv
        rel_range = 0.8
        a = 18
        b = 0.1
        # mouse ignorieren
        size_mouse = 0.8

        force = np.zeros(2)

        while self.sensor_ranges is None and self.sensor_angles is None:
            print("Wait for laser_callback.")

        rad_cheese = m.sqrt(2*0.1**2) #radius Mous
        scam_angle_increment=0.0175019223243
        
        distance_cat_mouse = self.distance(self.x_cat, self.y_cat, self.x_mouse, self.y_mouse)
        
        #Mouse rausrechnen
        if (self.state == "hunt" and distance_cat_mouse*size_mouse <= rel_range): # jagt und Mosu in sensorreichweite
            delta_x = self.x_mouse- self.x_cat
            delta_y = self.y_mouse - self.y_cat

            angle_mous = m.atan2(delta_y, delta_x) - self.phi_cat

            scan_mous_middle = round(((angle_mous)+2*m.pi)%(2*m.pi)/scam_angle_increment)*scam_angle_increment
            scan_mous_border = round(m.atan2(size_mouse, distance_cat_mouse)/scam_angle_increment)*scam_angle_increment

            border__mouse_l = abs(scan_mous_middle+scan_mous_border)%(2*m.pi)

            border__mouse_r = abs(scan_mous_middle-scan_mous_border)%(2*m.pi)

            print("Mous in Scanreichweite.")
            print("Ignor sensor_angles ",border__mouse_l, " bis ", border__mouse_r)

            for i in range(-rel_phi, rel_phi+1):
                print("I",i ," = ", self.sensor_angles[i], " R = ", self.sensor_ranges[i] )

                if(border__mouse_l < border__mouse_r): # geht ueber 0gard
                    print("A")
                    print("Winkel", self.sensor_angles[i])
                    if( self.sensor_angles[i] <= border__mouse_l):
                        print("Left")
                        print("Winkel", self.sensor_angles[i], "=0")
                        self.sensor_ranges[i] = np.inf
                    elif( self.sensor_angles[i] >= border__mouse_r):
                        print("right")
                        print("Winkel", self.sensor_angles[i], "=0")
                        self.sensor_ranges[i] = np.inf
                else:
                    print("B")
                    if( border__mouse_l >= self.sensor_angles[i] and self.sensor_angles[i]  >= border__mouse_r ):
                        print("Winkel", self.sensor_angles[i], "=0")
                        self.sensor_ranges[i] = np.inf


        distance_cheese = self.distance(self.x_cat, self.y_cat, self.x_cheese, self.y_cheese)
        # Cheese einrechnen
        if(distance_cheese-rad_cheese <= rel_range):
            delta_x = self.x_cheese- self.x_cat
            delta_y = self.y_cheese - self.y_cat

            angular_cheese = m.atan2(delta_y, delta_x) - self.phi_cat

            scan_cheese_middle = round(((angular_cheese)+2*m.pi)%(2*m.pi)/scam_angle_increment)*scam_angle_increment
            scan_cheese_border = round(m.atan2(rad_cheese, distance_cheese )/scam_angle_increment)*scam_angle_increment
            border_cheese_l = ((scan_cheese_middle+scan_cheese_border)+2*m.pi)%(2*m.pi)
            border_cheese_r = ((scan_cheese_middle-scan_cheese_border)+2*m.pi)%(2*m.pi)

            print("Cheese in Scanreichweite.")
            print("Relevante sensor_angles ",border_cheese_l, " bis ", border_cheese_r)

            for i in range(-rel_phi, rel_phi+1):
                if(border_cheese_l < border_cheese_r): # geht ueber 0gard
                    if(self.sensor_angles[i] <= border_cheese_l):
                        print("Midi", scan_cheese_middle, " Winkel", self.sensor_angles [i])
                        self.sensor_ranges[i] = distance_cheese-(m.sqrt (abs(rad_cheese**2 - (m.tan (abs(scan_cheese_middle-self.sensor_angles[i])) / distance_cheese) **2)))
                    elif( self.sensor_angles[i] >= border_cheese_r ):
                        print("Midi", scan_cheese_middle, " Winkel", self.sensor_angles [i])
                        self.sensor_ranges[i] = distance_cheese-(m.sqrt (abs(rad_cheese**2 - (m.tan(abs(scan_cheese_middle-self.sensor_angles[i]))/distance_cheese) **2)))
                elif( border_cheese_l > self.sensor_angles[i] > border_cheese_r ):
                    print("Midi", scan_cheese_middle, " Winkel", self.sensor_angles [i])
                    self.sensor_ranges[i] = distance_cheese-(m.sqrt (abs(rad_cheese**2-(m.tan (abs(scan_cheese_middle-self.sensor_angles[i]))/distance_cheese)**2)))

        # Force Berechnen
        for i in range(-rel_phi, rel_phi+1):
            if (self.sensor_ranges[i] < rel_range):
                force[1] += -m.sin(self.sensor_angles[i]) * (rel_range - self.sensor_ranges[i])


        force[1] /= a
        # if ((force[1] > -b and force[1] < b)):
        #     force[1] *= 2
        return force

    def homing(self):

        out = Twist()
        out.linear.x = CONSTANT_CAT_SPEED  # k_rho_0 * self.rho # + force[0]

        force = self.calculate_force()
        # print(force)

        # if self.distance(self.x_cat,self.y_cat,self.x_mouse,self.y_mouse) < 1:
        #     force[1] = 0
        print("ALPHA= ",self.alpha," Force= ",3*force[1])
        out.angular.z = self.alpha + 3*force[1]
        if out.angular.z > 0.8:
            out.angular.z = 0.8
        if out.angular.z < -0.8:
            out.angular.z = -0.8
        self.cat_publisher.publish(out)

    def set_state(self):

        distance_mouse_cat = self.distance(self.x_mouse, self.y_mouse, self.x_cat, self.y_cat)
        distance_mouse_cheese = self.distance(self.x_mouse, self.y_mouse, self.x_cheese, self.y_cheese)

        print(self.state)
        if self.state == "cheese":
            self.update_polar(self.x_cheese, self.y_cheese)
            if self.distance(self.x_cheese, self.y_cheese, self.x_cat, self.y_cat) < GOAL_RADIUS:
                self.state = "mid"
            if distance_mouse_cat < HUNT_RADIUS and distance_mouse_cheese > LET_GO_RADIUS:
                self.state = "hunt"
        elif self.state == "mid":
            self.calc_mid_point()
            self.update_polar(self.attraction_point_x, self.attraction_point_y)
            if self.distance(self.attraction_point_x, self.attraction_point_y, self.x_cat, self.y_cat) < GOAL_RADIUS:
                self.state = "hunt"
            if distance_mouse_cat < HUNT_RADIUS and distance_mouse_cheese < LET_GO_RADIUS:
                self.state = "hunt"
        elif self.state == "hunt":
            self.calc_preditction_mouse()
            self.update_polar(self.attraction_point_x, self.attraction_point_y)
            dist_mc = self.distance(self.x_mouse, self.y_mouse, self.x_cheese, self.y_cheese)
            dist_cc = self.distance(self.x_cat, self.y_cat, self.x_cheese, self.y_cheese)
            if dist_cc > dist_mc + HYSTERESIS:
                self.state = "cheese"
        self.homing()


if __name__ == '__main__':
    rospy.init_node('cat', anonymous=True)
    try:
        node = Cat()
    except rospy.ROSInterruptException:
        pass
