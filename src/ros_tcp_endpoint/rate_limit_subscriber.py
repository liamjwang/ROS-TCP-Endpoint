#  Copyright 2020 Unity Technologies
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rospy
import socket
import re

from .subscriber import RosSubscriber
from .fps import FPS


class RateLimitSubscriber(RosSubscriber):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, tcp_server, queue_size=10, rate_hz=0):
        super(RateLimitSubscriber, self).__init__(topic, message_class, tcp_server, queue_size)
        self.last_publish_time = rospy.Time.now()
        self.rate_hz = rate_hz
        self.timer = None
        self.timer_running = False
        self.latest_msg = None
        self.fps = FPS(printevery=1, name=self.topic)
    #     """

    #     Args:
    #         topic:         Topic name to publish messages to
    #         message_class: The message class in catkin workspace
    #         queue_size:    Max number of entries to maintain in an outgoing queue
    #     """
    #     strippedTopic = re.sub("[^A-Za-z0-9_]+", "", topic)
    #     self.node_name = "{}_RosSubscriber".format(strippedTopic)
    #     RosReceiver.__init__(self, self.node_name)
    #     self.topic = topic
    #     self.msg = message_class
    #     self.tcp_server = tcp_server
    #     self.queue_size = queue_size

    #     # Start Subscriber listener function
    #     self.sub = rospy.Subscriber(self.topic, self.msg, self.send)

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network

        """
        if self.rate_hz == 0:
            self.send_latest_msg()
            return

        self.latest_msg = data

        # the timer is going to publish the latest message
        if self.timer_running:
            return

        # timer is not running and there has been no message for a while
        curr_time = rospy.Time.now()
        if curr_time - self.last_publish_time > rospy.Duration(1.0 / self.rate_hz):
            self.send_latest_msg()
            return
        
        # timer is not running but there has been a recent message
        self.timer_running = True
        rospy.Timer(rospy.Duration(1/self.rate_hz), self.send_latest_msg, oneshot=True)
        return

    def send_latest_msg(self):
        if self.latest_msg is not None and self.sub is not None:
            self.tcp_server.send_unity_message(self.topic, self.latest_msg)
            self.fps()
            self.last_publish_time = rospy.Time.now()
            self.latest_msg = None
        self.timer_running = False

    # def unregister(self):
    #     """

    #     Returns:

    #     """
    #     if not self.sub is None:
    #         self.sub.unregister()
