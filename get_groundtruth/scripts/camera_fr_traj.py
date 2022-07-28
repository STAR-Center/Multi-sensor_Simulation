#!/usr/bin/env python
 
import roslib
import rospy
import math
import numpy
import tf
import tf.transformations as tft
import geometry_msgs.msg
from gazebo_msgs.srv import GetLinkState 
from gazebo_msgs.msg import LinkState
#from gazebo_msgs.msg import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
#from gazebo_msgs.msg import *
#import gazebo_msg.srv 
 
if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    get_state_service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    rate = rospy.Rate(50.0)
    pub = rospy.Publisher('/link4_under_aptag0', PoseStamped, queue_size=1)
    while not rospy.is_shutdown():
        try:
            t, r = listener.lookupTransform('/link4', '/camera_fr_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.error("failed to listen transform from '/link4' to '/camera_fr_link'")
            continue
        offset_to_link4 = numpy.matrix(tft.quaternion_matrix(r))
        offset_to_link4[0,3] = t[0]
        offset_to_link4[1,3] = t[1]
        offset_to_link4[2,3] = t[2]
        print('offset_to_link4 = ',offset_to_link4)
        
        rospy.wait_for_service('/gazebo/get_link_state')
        #link = GetLinkStateRequest()
        #link = LinkState()
        #link.link_name = 'link4'
        #link.reference_frame = 'Apriltag0'
        links = get_state_service("link4", "Apriltag0")
        linkstate = links.link_state
        link4matrix = tft.quaternion_matrix(numpy.array([linkstate.pose.orientation.x, linkstate.pose.orientation.y, linkstate.pose.orientation.z, linkstate.pose.orientation.w]))
        link4matrix[0,3] = linkstate.pose.position.x
        link4matrix[1,3] = linkstate.pose.position.y
        link4matrix[2,3] = linkstate.pose.position.z
        
        T = numpy.array(numpy.dot(link4matrix, offset_to_link4))
        print('T = ',T)
        
        
        p = Pose()
        # # Belows to check the correctness
        # R=T[:3,:3].transpose()
        # t = numpy.array([[-T[0,3]], [-T[1,3]], [-T[2,3]]])
        # t = R.dot(t)
        # p.position.x = t[0]
        # p.position.y = t[1]
        # p.position.z = t[2]
        # ps.header.frame_id = "camera_fr_link"
        p.position.x = T[0,3]
        p.position.y = T[1,3]
        p.position.z = T[2,3]
        R=T[:3,:3]
        roll, pitch, yaw = tft.euler_from_matrix(R)
        q = tft.quaternion_from_euler(roll, pitch, yaw)
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        ps = PoseStamped()
        ps.pose = p
        ps.header.stamp =  rospy.Time.now()
        ps.header.frame_id = "Apriltag0"
        pub.publish(ps)
        #tf_brocast(ps,q)

        rate.sleep()
