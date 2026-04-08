"""
Subscribes to odometry for multiple drones and publishes consolidated TF.
Avoids the time-jump issue caused by multiple Gazebo pose bridges.
"""

from functools import partial

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class OdomTfPublisher(Node):

    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.declare_parameter('drone_names', ['cf1', 'cf2', 'cf3', 'cf4'])
        drone_names = self.get_parameter('drone_names').value

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        # tf_static requires TRANSIENT_LOCAL durability
        static_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', static_qos)

        # Subscribe to each drone's odometry
        for name in drone_names:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                partial(self._odom_cb, name=name),
                10)

        # Publish static world -> cfX/odom transforms
        static_msg = TFMessage()
        for name in drone_names:
            t = TransformStamped()
            t.header.frame_id = 'world'
            t.child_frame_id = f'{name}/odom'
            t.transform.rotation.w = 1.0
            static_msg.transforms.append(t)
        # Publish periodically since tf_static needs at least one publish
        self._static_msg = static_msg
        self.create_timer(1.0, self._publish_static)
        self._publish_static()

        self.get_logger().info(
            f'Publishing TF for {len(drone_names)} drones: {drone_names}')

    def _publish_static(self):
        now = self.get_clock().now().to_msg()
        for t in self._static_msg.transforms:
            t.header.stamp = now
        self.tf_static_pub.publish(self._static_msg)

    def _odom_cb(self, msg: Odometry, name: str):
        t = TransformStamped()
        # Use the odometry message timestamp (from Gazebo via ros_gz_bridge)
        # to stay consistent with sim time and avoid time-jump warnings in RViz
        t.header.stamp = msg.header.stamp
        t.header.frame_id = f'{name}/odom'
        t.child_frame_id = f'{name}/base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Also publish base_footprint -> cfX (identity) so Crazyswarm2's
        # robot description (which uses link name "cfX") can find its frame
        t2 = TransformStamped()
        t2.header.stamp = t.header.stamp
        t2.header.frame_id = f'{name}/base_footprint'
        t2.child_frame_id = name
        t2.transform.rotation.w = 1.0

        tf_msg = TFMessage()
        tf_msg.transforms.append(t)
        tf_msg.transforms.append(t2)
        self.tf_pub.publish(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
