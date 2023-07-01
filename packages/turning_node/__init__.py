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

        # Construct subscriber for FSM state
        self.sub_fsm_state = rospy.Subscriber(
            "~fsm_node/fsm_state",
            FSMState,
            self.callbackFSMState,
            queue_size=1
        )

        self.log("Initialized!")

    def callbackFSMState(self, fsm_state_msg):
        """Callback for receiving the FSM state message.

        Args:
            fsm_state_msg (:obj:`FSMState`): Message containing the current FSM state.
        """
        self.fsm_state = fsm_state_msg.state

        if self.fsm_state == "TURNING":
            # Call the turning function here
            self.executeTurningFunction()

    def executeTurningFunction(self):
        """Execute the turning function."""
        # Implement your turning logic here
        # This function will be called when the state is TURNING
        self.log("Executing turning function.")

    def onShutdown(self):
        """Callback function to execute on shutdown."""
        rospy.loginfo("Shutting down.")


if __name__ == "__main__":
    # Initialize the node
    turning_node = TurningNode(node_name="turning_node")
    # Keep it spinning
    rospy.spin()
