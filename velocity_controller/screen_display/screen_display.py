import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyautogui

class DisplayScoreReportNode(Node):
    def __init__(self):
        super().__init__('display_score_report_node')
        self.subscription = self.create_subscription(String, 'score_report', self.callback, 10)

    def callback(self, msg):
        score_text = msg.data
        self.display_text_on_screen(score_text)

    def display_text_on_screen(self, text):
        screen_width, screen_height = pyautogui.size()
        text_width, text_height = pyautogui.textsize(text)
        x_position = screen_width - text_width - 10
        y_position = 10
        pyautogui.alert(text, title="Score Report", timeout=5, location=(x_position, y_position))

def main(args=None):
    rclpy.init(args=args)
    node = DisplayScoreReportNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

