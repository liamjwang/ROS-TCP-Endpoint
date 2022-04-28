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
import threading
import Queue as queue

from .rate import Rate
from .subscriber import RosSubscriber
from .fps import FPS


class RateLimitSubscriber(RosSubscriber):
    """
    Class to send messages outside of ROS network
    """

    def __init__(self, topic, message_class, tcp_server, queue_size=10, rate_hz=0):
        super(RateLimitSubscriber, self).__init__(topic, message_class, tcp_server, queue_size)
        if rate_hz == 0:
            rate_hz = 10
        self.rate_hz = rate_hz
        self.fps = FPS(printevery=5, name=self.topic)
        self.queue = queue.Queue()
        self.halt_event = threading.Event()
        threading.Thread(target=self.send_ratelimited_loop).start()

    def send(self, data):
        """
        Connect to TCP endpoint on client and pass along message
        Args:
            data: message data to send outside of ROS network


        """

        self.queue.put(data)
        return self.msg

    def send_ratelimited_loop(self, _=None):
        r = Rate(self.rate_hz)
        while not self.halt_event.is_set():
            try:
                msg = self.queue.get()
            except queue.Empty:
                continue
            try:
                for _ in range(1000):
                    msg = self.queue.get(block=False)
            except queue.Empty:
                pass
            self.tcp_server.send_unity_message(self.topic, msg)
            self.fps()
            r.sleep()

    def unregister(self):
        """

        Returns:

        """
        self.halt_event.set()
        if not self.sub is None:
            self.sub.unregister()
