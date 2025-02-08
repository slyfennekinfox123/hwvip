import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Parameters
        self.declare_parameters(namespace='',
            parameters=[
                ('front_angle_threshold', 0.1745),
                ('obstacle_distance', 1),
                ('linear_speed', 0.2),
                ('angular_speed', 0.5)
            ])
        
        # State machine
        self.state = 'moving'
        self.publish_ackermann(0.0, 0.2)        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Continuous command timer
        self.cmd_timer = self.create_timer(0.1, self.publish_continuous_cmd)
        self.get_logger().info("Node initialized")

    def scan_callback(self, msg):
        front_angle = self.get_parameter('front_angle_threshold').value
        obstacle_dist = self.get_parameter('obstacle_distance').value
        
        # Find ranges in front sector
        front_ranges = [
            d for i, d in enumerate(msg.ranges)
            if abs(msg.angle_min + i * msg.angle_increment) <= front_angle
            and msg.range_min <= d <= msg.range_max
        ]
        # State transitions
        obstacle_nearby = any(d < obstacle_dist for d in front_ranges)
        
        if self.state == 'moving' and obstacle_nearby:
            self.state = 'turning'
            self.get_logger().warning("OBSTACLE DETECTED! Turning left")
        elif self.state == 'turning' and not obstacle_nearby:
            self.state = 'moving'
            self.get_logger().info("Path clear - resuming forward motion")

    def publish_continuous_cmd(self):        
        if self.state == 'moving':
            self.publish_ackermann(0.0,0.2)
        else:
            self.publish_ackermann(-90, 0.05)       

    def publish_ackermann(self, steer_angle, velocity=0.0):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rclpy.time.Time().to_msg()
        ack_msg.header.frame_id = "laser"
        ack_msg.drive.steering_angle = steer_angle
        ack_msg.drive.speed = velocity

        self.ack_publisher.publish(ack_msg)     
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()