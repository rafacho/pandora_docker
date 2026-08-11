#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

import Queue
# import tf 
# from message_filters import SimpleFilter

import message_filters
import rospy
from sensor_msgs.msg import Image

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

import numpy as np
from pyquaternion import Quaternion
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose

class TfMessageFilter(SimpleFilter):
    """Stores a message unless corresponding transforms is 
    available
    """
    def __init__(self, input_filter, base_frame, target_frame, queue_size=500):
        SimpleFilter.__init__(self)
        self.connectInput(input_filter)
        self.base_frame = base_frame
        self.target_frame = target_frame
        # TODO: Use a better data structure
        self.message_queue = Queue.Queue(maxsize=queue_size)
        self.listener = tf.TransformListener()
        self.max_queue_size = queue_size
        self._max_queue_size_so_far = 0

    def connectInput(self, input_filter):
        self.incoming_connection = \
                input_filter.registerCallback(self.input_callback)

    def poll_transforms(self, latest_msg_tstamp):
        """
        Poll transforms corresponding to all messages. If found throw older
        messages than the timestamp of transform just found
        and if not found keep all the messages.
        """
        # Check all the messages for transform availability
        tmp_queue = Queue.Queue(self.max_queue_size)
        first_iter = True
        # Loop from old to new
        while not self.message_queue.empty():
            msg = self.message_queue.get()
            tstamp = msg.header.stamp
            if (first_iter and 
                self.message_queue.qsize() > self._max_queue_size_so_far):
                first_iter = False
                self._max_queue_size_so_far = self.message_queue.qsize()
                rospy.logdebug("Queue(%d) time range: %f - %f" %
                              (self.message_queue.qsize(), 
                               tstamp.secs, latest_msg_tstamp.secs))
                rospy.loginfo("Maximum queue size used:%d" %
                              self._max_queue_size_so_far)
            if self.listener.canTransform(self.base_frame, self.target_frame,
                                          tstamp):
                (trans, quat) = self.listener.lookupTransform(self.base_frame,
                                              self.target_frame, tstamp)
                self.signalMessage(msg, (trans, quat))
                # Note that we are deliberately throwing away the messages
                # older than transform we just received
                return
            else:
                # if we don't find any transform we will have to recycle all
                # the messages
                tmp_queue.put(msg)
        self.message_queue = tmp_queue

    def input_callback(self, msg):
        """ Handles incoming message """
        if self.message_queue.full():
            # throw away the oldest message
            rospy.logwarn("Queue too small. If you this message too often" + " consider increasing queue_size")
            self.message_queue.get()

        self.message_queue.put(msg)
        # This can be part of another timer thread
        # TODO: call this only when a new/changed transform
        self.poll_transforms(msg.header.stamp)

########################################################################################
#Usage



def callback(image, trans_rot):
    print("Got image:%f" % image.header.stamp.secs)

rospy.init_node('test_tf_sync_msgs', log_level=rospy.DEBUG)
wheel1_sub = message_filters.Subscriber('/pandora/contacts_1', ContactsState)

ts = TfMessageFilter(wheel1_sub, '/world', '/wheel_1', queue_size=1000)
ts.registerCallback(callback)
rospy.spin()
