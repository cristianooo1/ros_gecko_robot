import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Twist

topic1 = 'cmd_vel'

rate_msg = 2

def main(args=None):
    rclpy.init(args=args)
    
    controlVel = Twist()

    controlVel.linear.x = 4.0;
    controlVel.linear.y = 0.0;
    controlVel.linear.z = 0.0;
    controlVel.angular.x = 0.0;
    controlVel.angular.y = 0.0;
    controlVel.angular.z = 8.0;
    
    test_node = Node("test_node")
    publisher = test_node.create_publisher(Twist, topic1, 1)

    rate = test_node.create_rate(rate_msg)

    while rclpy.ok():
        print("sending control message")
        publisher.publish(controlVel)
        rclpy.spin_once(test_node)
        rate.sleep()

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()