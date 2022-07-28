#! /usr/bin/env python

import rospy, sys
# import std_msgs
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from math import sqrt
from math import pi
import moveit_commander
import threading
import random
import time

class MovePlatform:
    def __init__(self):
        rospy.init_node('move_platform', anonymous=True)
        # rospy.on_shutdown(self.shutdown)
        self.rate = 50
        self.r = rospy.Rate(self.rate)

        self.cmd_x = rospy.Publisher('/multi_sensor_platform/x_axis_controller/command', Float64, queue_size=5)
        self.cmd_y = rospy.Publisher('/multi_sensor_platform/y_axis_controller/command', Float64, queue_size=5)
        self.cmd_z = rospy.Publisher('/multi_sensor_platform/up_down_controller/command', Float64, queue_size=5)
        self.cmd_yaw = rospy.Publisher('/multi_sensor_platform/yaw_controller/command', Float64, queue_size=5)
        self.cmd_roll = rospy.Publisher('/multi_sensor_platform/roll_controller/command', Float64, queue_size=5)
        self.cmd_pitch = rospy.Publisher('/multi_sensor_platform/pitch_controller/command', Float64, queue_size=5)

        rospy.Subscriber("/multi_sensor_platform/joint_states", JointState, self.jointstate_callback)

        self.linear_acc_limit = 1.5
        self.angular_acc_limit = 2.5
        self.linear_speed_x = 1.0
        self.linear_speed_y = 1.0
        self.linear_speed_z = 1.0
        self.yaw_speed = 1.0
        self.roll_speed = 1.0
        self.pitch_speed = 1.0

        # position of: x y z roll pich yaw
        self.joint_p = [0.0, 0.0, -0.5, 0.0, 0.0, 0.0]
        # velocity of: x y z
        self.joint_v = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.goal_tolerance_z = 0.02
        self.goal_tolerance_xy = 0.3 # 0.1
        self.vel_tolerance = 0.005
        self.ang_tolerance = 0.015
        self.linear_effort = 0.5

        self.xyz_itr = 150
        self.rpy_itr = 150
        self.xyz_finished=False
        self.rpy_finished=False

        self._set_linear_effort(-0.25, self.cmd_z, 2, self.goal_tolerance_z, 50, 10)
        time.sleep(3)
        key = raw_input('finish __init__, press any number key...')
        print('start motions')

    def jointstate_callback(self, joint_msgs):
        self.joint_v[0] = joint_msgs.velocity[4]
        self.joint_v[1] = joint_msgs.velocity[5]
        self.joint_v[2] = joint_msgs.velocity[1]
        self.joint_v[3] = joint_msgs.velocity[3]
        self.joint_v[4] = joint_msgs.velocity[2]
        self.joint_v[5] = joint_msgs.velocity[0]
        self.joint_p[0] = joint_msgs.position[4]
        self.joint_p[1] = joint_msgs.position[5]
        self.joint_p[2] = joint_msgs.position[1]
        self.joint_p[3] = joint_msgs.position[3]
        self.joint_p[4] = joint_msgs.position[2]
        self.joint_p[5] = joint_msgs.position[0]
        # for i in range(3):
        #     print('joint ', i, ' position: ',self.joint_p[i])

    def _set_linear_effort(self, goal_p, pub_msg, j, goal_tolerance, balance_effort =0, effort_factor=1.0):
        if abs(goal_p - self.joint_p[j])<goal_tolerance:
            return
        effort_direction = (goal_p>self.joint_p[j])*2.0-1.0
        curr_eff = balance_effort + effort_direction*effort_factor*self.linear_effort
        # print('curr effort: ',curr_eff)
        while abs(goal_p - self.joint_p[j])>goal_tolerance:
            if j==0 and self.joint_p[1]*self.joint_p[1]+self.joint_p[0]*self.joint_p[0]>0.54+self.goal_tolerance_xy:
                print('out of circle')
                self.r.sleep()
                break
            # if j==0 and self.joint_p[1]*self.joint_p[1]+goal_p*goal_p>1.0:
            #     print(self.joint_p[1], ' + ', goal_p)
            #     goal_sign = (goal_p>0)*2-1.0
            #     p_y= min(self.joint_p[1], 1.0)
            #     p_y= max(p_y, -1.0)
            #     goal_p= goal_sign * sqrt(1.0-p_y*p_y)
            move_eff = curr_eff
            pub_msg.publish(move_eff)
            # print('curr position for joint ', j, ' is: ', self.joint_p[j])
            self.r.sleep()
        print('before decelerate')
        if balance_effort:  # for z axis: need no deceleration
            pub_msg.publish(balance_effort)
        else: # for x y axis: need deceleration
            self._set_linear_decelerate( pub_msg,j,10)


    def _set_linear_decelerate(self, pub_msg, j, effort_factor=50.0):
        # while effort_direction* (self.joint_v[j] - effort_direction*self.goal_tolerance)>0:
        while abs(self.joint_v[j])>self.vel_tolerance:
            curr_eff =  - self.joint_v[j]*effort_factor
            # curr_eff = balance_effort - ((self.joint_v[j]>0)*2-1.0)*effort_factor*self.linear_effort
            move_eff = curr_eff
            pub_msg.publish(move_eff)
            # print('self.joint_v[j] = ', self.joint_v[j])
            self.r.sleep()
        pub_msg.publish(0)


    def up_down(self):
        self._set_linear_effort(0.0, self.cmd_z, 2, self.goal_tolerance_z, 50, 10)
        self._set_linear_effort(-0.5, self.cmd_z, 2, self.goal_tolerance_z, 50, 10)
        self._set_linear_effort(-0.25, self.cmd_z, 2, self.goal_tolerance_z, 50, 10)
        print('finished up down')

    def left_right(self):
        self._set_linear_effort(0.5, self.cmd_x, 0, self.goal_tolerance_xy)
        self._set_linear_effort(-0.5, self.cmd_x, 0, self.goal_tolerance_xy)
        self._set_linear_effort(0.0, self.cmd_x, 0, self.goal_tolerance_xy)
        # self._set_linear_effort(-1.0, self.cmd_x, 0, self.goal_tolerance_xy)
        # self._set_linear_effort(0.0, self.cmd_x, 0, self.goal_tolerance_xy)
    def forward_backward(self):
        self._set_linear_effort(0.67, self.cmd_y, 1, self.goal_tolerance_xy)
        self._set_linear_effort(-0.1, self.cmd_y, 1, self.goal_tolerance_xy)
        # self._set_linear_effort(-1.0, self.cmd_y, 1, self.goal_tolerance_xy)
        self._set_linear_effort(0.2, self.cmd_y, 1, self.goal_tolerance_xy)


    def _set_angular_vel(self, pub_msg, j, goal_degree, vel = 0.15, goal_tolerance = 0.02):
        sign = (goal_degree > self.joint_p[j])*2.0-1.0
        ac_step =0.01
        ac_vel =0
        while j!=5 and ac_vel<vel:
            ac_vel += ac_step
            pub_msg.publish(sign*ac_vel)
            self.r.sleep()
        while abs(self.joint_p[j] - goal_degree) > goal_tolerance:
            # if j!=5:
            #     print('abs(self.joint_p[',j,'] - goal_degree) > goal_tolerance? ', abs(self.joint_p[j] - goal_degree) > goal_tolerance)
            pub_msg.publish(sign*vel)
            self.r.sleep()
        de_vel = vel
        while j!=5 and de_vel>0:
            de_vel -= ac_step
            pub_msg.publish(sign*de_vel)
            self.r.sleep()
        pub_msg.publish(0.0)
        while j!=5 and abs(self.joint_v[j])>self.ang_tolerance:
            pub_msg.publish(0.0)
            self.r.sleep()

    def yaw_pitch_roll(self):
        # yaw
        self._set_angular_vel(self.cmd_yaw, 5, 0.5, 0.23)
        self._set_angular_vel(self.cmd_yaw, 5, -0.5, 0.23)
        # self._set_angular_vel(self.cmd_yaw, 5, -1.57, 0.3)
        self._set_angular_vel(self.cmd_yaw, 5, 0.0, 0.23)

        # pitch
        self._set_angular_vel(self.cmd_pitch, 4, 0.2)
        self._set_angular_vel(self.cmd_pitch, 4, -0.3)
        self._set_angular_vel(self.cmd_pitch, 4, 0.0)

        # roll
        self._set_angular_vel(self.cmd_roll, 3, 0.2)
        self._set_angular_vel(self.cmd_roll, 3, -0.3)
        self._set_angular_vel(self.cmd_roll, 3, 0.0)

    def single_axis_motions(self):
        self.up_down()
        self.forward_backward()
        self.left_right()
        #
        self.yaw_pitch_roll()

    def random_x(self):
        for i in range(self.xyz_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            posi_x = random.randint(-5,5)/10.0
            print('random x = ', posi_x)
            self._set_linear_effort(posi_x, self.cmd_x, 0, self.goal_tolerance_xy)
        print("random xyz finished")
        self.xyz_finished=True
        return

    def random_y(self):
        for i in range(self.xyz_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            posi_y = random.randint(-1,5)/10.0
            print('random y = ', posi_y)
            self._set_linear_effort(posi_y, self.cmd_y, 1, self.goal_tolerance_xy)
        print("random xyz finished")
        self.xyz_finished=True
        return
    def random_z(self):
        for i in range(self.xyz_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            posi_z = -0.25 + random.randint(-5,25)/100.0
            print('random z = ', posi_z)
            self._set_linear_effort(posi_z, self.cmd_z, 2, self.goal_tolerance_z, 50, 10)
        print("random xyz finished")
        self.xyz_finished=True
        return

    def random_roll(self):
        for i in range(self.rpy_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            r = random.randint(-30,20)/100.0
            print('random roll = ',r, ' ... ', self.joint_p[3])
            self._set_angular_vel(self.cmd_roll, 3, r)
        print("random rpy finished")
        self.rpy_finished=True
        return
    def random_pitch(self):
        for i in range(self.rpy_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            p = random.randint(-30,20)/100.0
            print('random pitch = ',p, ' ... ', self.joint_p[4])
            self._set_angular_vel(self.cmd_pitch, 4, p)
        print("random rpy finished")
        self.rpy_finished=True
        return

    def random_yaw(self):
        for i in range(self.rpy_itr):
            if self.rpy_finished or self.xyz_finished:
                return
            y = random.randint(-50, 50)/100.0
            print('random yaw = ',y)
            self._set_angular_vel(self.cmd_yaw, 5, y, 0.3)
        print("random rpy finished")
        self.rpy_finished=True
        return

    def shutdown(self):
        rospy.loginfo("Stopping the robot")
        self.cmd_x.publish(0)
        self.cmd_y.publish(0)
        self.cmd_z.publish(50)
        self.cmd_roll.publish(0)
        self.cmd_pitch.publish(0)
        self.cmd_yaw.publish(0)
        rospy.sleep(1)
        return

if __name__=='__main__':
    try:
        a = MovePlatform()

        # step_1: motions on each axis
        a.single_axis_motions()

        # step_2: random motions
        thread1 = threading.Thread(target=a.random_x,)
        thread1.start()
        thread2 = threading.Thread(target=a.random_y,)
        thread2.start()
        thread3 = threading.Thread(target=a.random_z,)
        thread3.start()

        thread4 = threading.Thread(target=a.random_roll,)
        thread4.start()
        thread5 = threading.Thread(target=a.random_pitch,)
        thread5.start()
        thread6 = threading.Thread(target=a.random_yaw,)
        thread6.start()

    except:
        rospy.loginfo("Platform moving node terminated by exception")
