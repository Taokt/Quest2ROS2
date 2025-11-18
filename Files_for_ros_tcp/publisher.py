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

import rclpy
import re

from rclpy.serialization import deserialize_message

from .communication import RosSender
from ros_tcp_endpoint.ros_msg_converter import convert_data


class RosPublisher(RosSender):
    """
    Class to publish messages to a ROS topic
    """

    # TODO: surface latch functionality
    def __init__(self, topic, message_class, queue_size=10, latch=False):
        """

        Args:
            topic:         Topic name to publish messages to
            message_class: The message class in catkin workspace
            queue_size:    Max number of entries to maintain in an outgoing queue
        """
        strippedTopic = re.sub("[^A-Za-z0-9_]+", "", topic)
        node_name = f"{strippedTopic}_RosPublisher"
        RosSender.__init__(self, node_name)
        self.topic = topic  
        self.msg = message_class()
        self.pub = self.create_publisher(message_class, topic, queue_size)

    def send(self, data):
        """
        Takes in serialized message data from source outside of the ROS network,
        deserializes it into it's message class, and publishes the message to ROS topic.

        Args:
            data: The already serialized message_class data coming from outside of ROS

        Returns:
            None: Explicitly return None so behaviour can be
        """

        try:
            msg = convert_data(self.topic, data)  
            if msg:
                print(f"[DEBUG] Publishing parsed message to {self.topic}")
                self.pub.publish(msg)
            else:
                print(f"[WARNING] Could not convert data for topic {self.topic}")
        except Exception as e:
            print(f"[ERROR] Failed to process message for topic {self.topic}: {e}")

        return None

    def unregister(self):
        """

        Returns:

        """
        self.destroy_publisher(self.pub)
        self.destroy_node()
