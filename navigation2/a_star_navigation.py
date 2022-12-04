import rclpy
from rclpy.node import Node
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from geometry_msgs.msg import Point, Twist, PoseStamped
import numpy as np


class NavStack(Node):
    def __init__(self):
        super().__init__('navstack')

        # Input Matrix [x,y] of Goal Waypoints
        self.pos = np.array([[1.497, 0.25],
                             [0.20, 0.20],
                             [1.6, 0.7]
                             ])
        self.ind = 0
        self.goal = self.pos[self.ind, :]

        self.orient = PoseStamped()

        self.orient.pose.position.x = self.goal[0]
        self.orient.pose.position.y = self.goal[1]
        self.orient.pose.position.z = 0.0
        self.orient.pose.orientation.x = 0.0
        self.orient.pose.orientation.y = 0.0
        self.orient.pose.orientation.z = 0.0
        self.orient.pose.orientation.w = 1.0

        self._pose_subscriber = self.create_subscription(NavigateToPose_FeedbackMessage,
                                                         '/navigate_to_pose/_action/feedback', self._motion_controller,
                                                         1)
        self._pose_subscriber

        self._velocity_publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self._velocity_publisher

        self._velocity_publisher.publish(self.orient)

    def _motion_controller(self, NavigateToPose_FeedbackMessage):
        msg = NavigateToPose_FeedbackMessage

        self.goal = self.pos[self.ind, :]
        print(self.goal)

        if msg.feedback.distance_remaining < 0.25:
            self.ind += 1
            self.goal = self.pos[self.ind, :]

        self.orient.pose.position.x = self.goal[0]
        self.orient.pose.position.y = self.goal[1]
        self.orient.pose.position.z = 0.0
        self.orient.pose.orientation.x = 0.0
        self.orient.pose.orientation.y = 0.0
        self.orient.pose.orientation.z = 0.0
        self.orient.pose.orientation.w = 0.0

        print(self.orient.pose.position.x)
        print(self.orient.pose.position.y)

        self._velocity_publisher.publish(self.orient)


def main():
    rclpy.init()
    obj = NavStack()
    while rclpy.ok():
        rclpy.spin_once(obj)
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()