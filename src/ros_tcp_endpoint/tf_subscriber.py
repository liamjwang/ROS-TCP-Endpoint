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

from ros_tcp_endpoint.fps import FPS
import rospy
import socket
import re

from .subscriber import RosSubscriber
# from tf2_msgs import TFMessage


class TfSubscriber(RosSubscriber):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, tcp_server, queue_size=10, rate_hz=0):
        super(TfSubscriber, self).__init__(topic, message_class, tcp_server, queue_size)
        self.last_publish_dict = {}
        self.fps_dict = {}
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

        Returns:
            self.msg: The deserialize message

        """
        key = data.transforms[0].child_frame_id
        if key in self.last_publish_dict:
            if data.transforms[0].header.stamp - self.last_publish_dict[key] < rospy.Duration(0.1):
                return self.msg
        if key not in self.fps_dict:
            self.fps_dict[key] = FPS(printevery=5, name="tf::"+key)
        self.fps_dict[key]()
        self.last_publish_dict[key] = data.transforms[0].header.stamp
        self.tcp_server.send_unity_message(self.topic, data)
        # print("send tf")
        return self.msg

    # def unregister(self):
    #     """

    #     Returns:

    #     """
    #     if not self.sub is None:
    #         self.sub.unregister()
