#!/usr/bin/env python3

import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import FSMState


class TurningNode(DTROS):
    """Node for executing turning function when the state is set to TURNING."""

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TurningNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Initialize variables
        self.fsm_state = None
        self.turning_speed = 0.2  # Adjust the turning speed as needed
        self.angle_accumulated = 0.0
        self.angle_to_turn = 2 * 3.14159  # 360 degrees in radians

        # Construct publisher for wheel commands
        self.pub_wheel_cmd = rospy.Publisher(
            "~wheel_command", Float32, queue_size=1, dt_topic_type=TopicType.CONTROL
        )

        self.log("Initialized!")

    def executeTurningFunction(self):
        """Execute the 360-degree turning function."""
        while self.angle_accumulated < self.angle_to_turn:
            # Publish wheel command to turn
            self.publishWheelCommand(self.turning_speed)
            # Sleep for a short duration
            rospy.sleep(0.1)
            # Update the accumulated angle
            self.angle_accumulated += abs(self.turning_speed * 0.1)

        # Stop the turning motion
        self.publishWheelCommand(0.0)
        self.log("Completed 360-degree turn.")

    def publishWheelCommand(self, speed):
        """Publish the wheel command for turning.

        Args:
            speed (:obj:`float`): The wheel turning speed.
        """
        wheel_command = Float32()
        wheel_command.data = speed
        self.pub_wheel_cmd.publish(wheel_command)

    def onShutdown(self):
        """Callback function to execute on shutdown."""
        self.publishWheelCommand(0.0)
        rospy.loginfo("Shutting down.")


if __name__ == "__main__":
    # Initialize the node
    turning_node = TurningNode(node_name="turning_node")
    # Execute the turning function
    turning_node.executeTurningFunction()
