import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')

        # Publisher object - (Message type, topic name, queue size)
        self.pub_ = self.create_publisher(String, 'chatter', 10)

        self.counter_= 0
        self.frequency_= 1
        self.get_logger().info('Publishing at %d Hz' %self.frequency_)

        self.timer = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):

        # Message 
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.counter_
        
        # Publish the message
        self.pub_.publish(msg)
        self.counter_ += 1

def main():
    rclpy.init()
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
