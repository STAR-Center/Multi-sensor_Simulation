#!/usr/bin/env python  
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import tf.transformations as tft
import geometry_msgs.msg
import numpy as np

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (tgt,rgt) = listener.lookupTransform('/velodyne_right', '/imu_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        T_gt = np.matrix(tft.quaternion_matrix(rgt))
        Rgt = T_gt[:3,:3]
        T_gt[:3,3] = np.array([tgt]).reshape(-1,1)
        #print('trans = ', tgt)
        #print('rot=', Rgt)
        print('Tgt = ',T_gt)
        
        #r_add = np.array([0.01, 0.04, -0.02]) #2.5deg=0.04rad
        #t_add = np.array([-0.04, 0.15, -0.09])
        r_add = np.array([-0.015, 0.03, 0.025]) #2.5deg=0.04rad
        t_add = np.array([0.07, -0.05, -0.013])
        T_add = np.matrix(tft.quaternion_matrix(tft.quaternion_from_euler(-0.015, 0.03, 0.025)))
        T_add[:3,3] = np.array([t_add]).reshape(-1,1)
        print('T_add = ', T_add)
        
        T_init= np.dot(T_add, T_gt)
        print('T_init = ')
        print(T_init)

        rate.sleep()
