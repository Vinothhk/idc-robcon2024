import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import curses

class KeyboardControlNode(Node):
    def __init__(self, stdscr):
        super().__init__('keyboard_control_node')
        self.stdscr = stdscr
        self.get_logger().info("Keyboard Control Node has been started. Press 'q' to quit.")
        
        # 创建一个Publisher，发布到'topic_name'主题
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_arm = self.create_publisher(Float64MultiArray, 'roboarm_joint_controller/commands', 10)
        
        self.linear = 0.0
        self.angular = 0.0
        self.arm_angel = [0., 0.]

        self.run()

    def run(self):
        while rclpy.ok():
            key = self.stdscr.getch()

            if key == ord('w'):
                self.move_forward()
            elif key == ord('x'):
                self.move_backward()
            elif key == ord('a'):
                self.turn_left()
            elif key == ord('d'):
                self.turn_right()
            elif key == ord('s'):
                self.stop()
            elif key == ord('j'):
                self.arm_up()
            elif key == ord('k'):
                self.arm_down()
            elif key == ord('q'):
                self.quit_program()
                break
            
            # 发布按键消息
            self.publish()

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.publisher_vel.publish(msg)
        self.get_logger().info(f"Published: {msg.linear}, {msg.angular}")
        msg = Float64MultiArray()
        msg.data = self.arm_angel
        self.publisher_arm.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def move_forward(self):
        self.get_logger().info("Moving forward\n")
        self.linear -= 0.1

    def move_backward(self):
        self.get_logger().info("Moving backward")
        self.linear += 0.1

    def turn_left(self):
        self.get_logger().info("Turning left")
        self.angular += 0.1

    def turn_right(self):
        self.get_logger().info("Turning right")
        self.angular -= 0.1

    def stop(self):
        self.get_logger().info("Stop")
        self.linear = 0.0
        self.angular = 0.0

    def quit_program(self):
        self.get_logger().info("Quitting program")
        rclpy.shutdown()

    def arm_up(self):
        self.arm_angel[0] -= 0.1
        self.arm_angel[1] += 0.11
        self.get_logger().info("Arm Up")

    def arm_down(self):
        self.arm_angel[0] += 0.1
        self.arm_angel[1] -= 0.11
        self.get_logger().info("Arm down")
        

def main(args=None):
    rclpy.init(args=args)

    # Initialize curses and start the node
    curses.wrapper(lambda stdscr: KeyboardControlNode(stdscr))

    rclpy.shutdown()

if __name__ == '__main__':
    main()
