import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Color

class CircleNode(Node):
    def __init__(self):
        # Init 
        super().__init__('circle_node')

        # Pub 
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        
        # Sub 
        self.subscription = self.create_subscription(
            Color,
            '/turtle1/color_sensor',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Init circle params
        self.get_logger().info('Circle Node has been started.')
        
    # Callback function to publish circle points
    def timer_callback(self):
        msg = Twist() 

        msg.linear.x = 0.5  
        msg.angular.z = 0.3  

        self.get_logger().info('Publishing circle command: linear.x=%.2f, angular.z=%.2f' % (msg.linear.x, msg.angular.z))
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.get_logger().info('Received color from /turtle1/color_sensor: r=%d, g=%d, b=%d' % (msg.r, msg.g, msg.b))    

def main():
    rclpy.init()

    circle_node = CircleNode()

    rclpy.spin(circle_node)
    circle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

