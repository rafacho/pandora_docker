#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import message_filters
import numpy as np
from pyquaternion import Quaternion
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from pandora_msgs.msg import WheelContacts


import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

def to_quaternion(trans):
	# return a normalized quaternion from tf2 transform
	qx = trans.transform.rotation.x
	qy = trans.transform.rotation.y
	qz = trans.transform.rotation.z
	qw = trans.transform.rotation.w

	q = Quaternion(qw, qx, qy, qz)
	qn = q.unit
	return qn

def callback(f1, f2, f3, f4):

	try:
		trans1 = tfBuffer.lookup_transform( 'world', 'wheel_1',f1.header.stamp, rospy.Duration(1.0))		
		trans2 = tfBuffer.lookup_transform( 'world', 'wheel_2',f2.header.stamp, rospy.Duration(1.0))
		trans3 = tfBuffer.lookup_transform( 'world', 'wheel_3',f3.header.stamp, rospy.Duration(1.0))
		trans4 = tfBuffer.lookup_transform( 'world', 'wheel_4',f4.header.stamp, rospy.Duration(1.0))

		if (trans1.header.stamp == f1.header.stamp) and (trans2.header.stamp == f2.header.stamp) and (trans3.header.stamp == f3.header.stamp) and (trans4.header.stamp == f4.header.stamp):
			
			q1n = to_quaternion(trans1)
			q2n = to_quaternion(trans2)
			q3n = to_quaternion(trans3)
			q4n = to_quaternion(trans4)

			# initialize f transformed
			f1t = f1
			f2t = f2
			f3t = f3
			f4t = f4
			wheelContacts = WheelContacts()
			wheelContacts.wheel1.wrenches.append(Wrench())
			wheelContacts.wheel1.total_wrench = Wrench()
			wheelContacts.wheel1.contact_positions.append(Vector3())
			wheelContacts.wheel1.contact_normals.append(Vector3())

			wheelContacts.wheel2.wrenches.append(Wrench())
			wheelContacts.wheel2.total_wrench = Wrench()
			wheelContacts.wheel2.contact_positions.append(Vector3())
			wheelContacts.wheel2.contact_normals.append(Vector3())

			wheelContacts.wheel3.wrenches.append(Wrench())
			wheelContacts.wheel3.total_wrench = Wrench()
			wheelContacts.wheel3.contact_positions.append(Vector3())
			wheelContacts.wheel3.contact_normals.append(Vector3())

			wheelContacts.wheel4.wrenches.append(Wrench())
			wheelContacts.wheel4.total_wrench = Wrench()
			wheelContacts.wheel4.contact_positions.append(Vector3())
			wheelContacts.wheel4.contact_normals.append(Vector3())
			# p = 0
			# fw = []
			
			# Rotates all contact forces to world base
			if f1.states:
				w1 = f1.states[0].total_wrench.force
				f1Rotated = q1n.rotate((w1.x, w1.y, w1.z))
				f1t.states[0].total_wrench.force.x = f1Rotated[0]
				f1t.states[0].total_wrench.force.y = f1Rotated[1]
				f1t.states[0].total_wrench.force.z = f1Rotated[2]
				
				wheelContacts.header = f1.header
				wheelContacts.header.frame_id = ""
				wheelContacts.wheel1 = f1t.states[0]
				
			if f2.states:
				w2 = f2.states[0].total_wrench.force
				f2Rotated = q2n.rotate((w2.x, w2.y, w2.z))
				f2t.states[0].total_wrench.force.x = f2Rotated[0]
				f2t.states[0].total_wrench.force.y = f2Rotated[1]
				f2t.states[0].total_wrench.force.z = f2Rotated[2]

				wheelContacts.wheel2 = f2t.states[0]
				
			if f3.states:
				w3 = f3.states[0].total_wrench.force
				f3Rotated = q3n.rotate((w3.x, w3.y, w3.z))
				f3t.states[0].total_wrench.force.x = f3Rotated[0]
				f3t.states[0].total_wrench.force.y = f3Rotated[1]
				f3t.states[0].total_wrench.force.z = f3Rotated[2]

				wheelContacts.wheel3 = f3t.states[0]
			
			if f4.states:
				w4 = f4.states[0].total_wrench.force
				f4Rotated = q4n.rotate((w4.x, w4.y, w4.z))
				f4t.states[0].total_wrench.force.x = 	f4Rotated[0]
				f4t.states[0].total_wrench.force.y = 	f4Rotated[1]
				f4t.states[0].total_wrench.force.z = 	f4Rotated[2]

				wheelContacts.wheel4 = f4t.states[0]
					
		else:
			rospy.logerr("state not found")

		# print(wheelContacts)
		wheelPub.publish(wheelContacts)
		

	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		rospy.logerr("Can not transform wheel contacts! ")
		
	rate.sleep()	

if __name__ == '__main__':
		
	rospy.init_node('wheel_forces', anonymous=False)
	rate = rospy.Rate(10) #10hz

	wheelPub = rospy.Publisher('/pandora/wheel_contacts', WheelContacts, queue_size=1)    

	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	wheel1_sub = message_filters.Subscriber('/pandora/contacts_1', ContactsState)
	wheel2_sub = message_filters.Subscriber('/pandora/contacts_2', ContactsState)
	wheel3_sub = message_filters.Subscriber('/pandora/contacts_3', ContactsState)
	wheel4_sub = message_filters.Subscriber('/pandora/contacts_4', ContactsState)

	ts = message_filters.ApproximateTimeSynchronizer([wheel1_sub, wheel2_sub, wheel3_sub, wheel4_sub], 1, 0.1)
	ts.registerCallback(callback)

	rospy.spin()