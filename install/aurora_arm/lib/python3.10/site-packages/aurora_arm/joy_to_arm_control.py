#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import time

class JoyToArmControl(Node):
    def __init__(self):
        super().__init__('joy_to_arm_control')
        
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        
        self.arm_pub = self.create_publisher(
            Float64MultiArray, '/arm_velocity_controller/commands', 10)
        
        self.hand_pub = self.create_publisher(
            Float64MultiArray, '/hand_velocity_controller/commands', 10)
        
        # ボタンの前の状態を記録（それぞれ独立した変数）
        self.prev_button_x = False
        self.prev_button_x_l1 = False        # L1+×用
        self.prev_button_circle = False      # ○用
        self.prev_button_circle_l1 = False   # L1+○用
        self.prev_button_triangle = False    # △用
        self.prev_button_triangle_l1 = False # L1+△用
        self.prev_button_square = False      # □用
        self.prev_button_square_l1 = False   # L1+□用

        # タイマーで制御
        self.timer = self.create_timer(0.1, self.control_timer)
        self.current_joy = None
        
        self.get_logger().info("Joy to Arm Control node started")

    def joy_callback(self, msg):
        self.current_joy = msg

    def control_timer(self):
        if self.current_joy is None:
            return
        
        msg = self.current_joy
        arm_msg = Float64MultiArray()
        arm_msg.data = [0.0, 0.0, 0.0, 0.0]  # rotation1, rotation2, rotation3, rotation4のデフォルト値

        # rotation1の制御
        rotation1_velocity = 0.0
        
        # バツボタン（ボタン0）でrotation1右回転
        if len(msg.buttons) > 0 and msg.buttons[0] and not (len(msg.buttons) > 4 and msg.buttons[4]):
            if not self.prev_button_x:
                self.get_logger().info("X button pressed - rotation1 right")
            rotation1_velocity = 1.0
            self.prev_button_x = True
        else:
            if self.prev_button_x:
                self.get_logger().info("X button released - rotation1 stopping")
            self.prev_button_x = False
        
        # L1+バツボタンでrotation1左回転
        if len(msg.buttons) > 4 and msg.buttons[0] and msg.buttons[4]:
            if not self.prev_button_x_l1:
                self.get_logger().info("L1+X button pressed - rotation1 left")
            rotation1_velocity = -1.0
            self.prev_button_x_l1 = True
        else:
            if self.prev_button_x_l1:
                self.get_logger().info("L1+X button released - rotation1 stopping")
            self.prev_button_x_l1 = False
        
        arm_msg.data[0] = rotation1_velocity
        
        # rotation2の制御
        rotation2_velocity = 0.0
        
        # ○ボタンでrotation2右回転
        if len(msg.buttons) > 1 and msg.buttons[1] and not (len(msg.buttons) > 4 and msg.buttons[4]):
            if not self.prev_button_circle:
                self.get_logger().info("Circle button pressed - rotation2 right")
            rotation2_velocity = 1.0
            self.prev_button_circle = True
        else:
            if self.prev_button_circle:
                self.get_logger().info("Circle button released - rotation2 stopping")
            self.prev_button_circle = False

        # L1+○ボタンでrotation2左回転
        if len(msg.buttons) > 4 and msg.buttons[1] and msg.buttons[4]:
            if not self.prev_button_circle_l1:
                self.get_logger().info("L1+Circle button pressed - rotation2 left")
            rotation2_velocity = -1.0
            self.prev_button_circle_l1 = True
        else:
            if self.prev_button_circle_l1:
                self.get_logger().info("L1+Circle button released - rotation2 stopping")
            self.prev_button_circle_l1 = False

        arm_msg.data[1] = rotation2_velocity


        rotation3_velocity = 0.0
        # △ボタンでrotation3右回転
        if len(msg.buttons) > 2 and msg.buttons[2] and not (len(msg.buttons) > 4 and msg.buttons[4]):
            if not self.prev_button_triangle:
                self.get_logger().info("Triangle button pressed - rotation3 right")
            rotation3_velocity = 1.0
            self.prev_button_triangle = True
        else:
            if self.prev_button_triangle:
                self.get_logger().info("Triangle button released - rotation3 stopping")
            self.prev_button_triangle = False

        # L1+△ボタンでrotation3左回転
        if len(msg.buttons) > 4 and msg.buttons[2] and msg.buttons[4]:
            if not self.prev_button_triangle_l1:
                self.get_logger().info("L1+Triangle button pressed - rotation3 left")
            rotation3_velocity = -1.0
            self.prev_button_triangle_l1 = True
        else:
            if self.prev_button_triangle_l1:
                self.get_logger().info("L1+Triangle button released - rotation3 stopping")
            self.prev_button_triangle_l1 = False

        arm_msg.data[2] = rotation3_velocity

        rotation4_velocity = 0.0
        # □ボタンでrotation4右回転
        if len(msg.buttons) > 3 and msg.buttons[3] and not (len(msg.buttons) > 4 and msg.buttons[4]):
            if not self.prev_button_square:
                self.get_logger().info("Square button pressed - rotation4 right")
            rotation4_velocity = 1.0
            self.prev_button_square = True
        else:
            if self.prev_button_square:
                self.get_logger().info("Square button released - rotation4 stopping")
            self.prev_button_square = False

        # L1+□ボタンでrotation4左回転
        if len(msg.buttons) > 4 and msg.buttons[3] and msg.buttons[4]:
            if not self.prev_button_square_l1:
                self.get_logger().info("L1+Square button pressed - rotation4 left")
            rotation4_velocity = -1.0
            self.prev_button_square_l1 = True
        else:
            if self.prev_button_square_l1:
                self.get_logger().info("L1+Square button released - rotation4 stopping")
            self.prev_button_square_l1 = False

        arm_msg.data[3] = rotation4_velocity


        # アーム制御を送信
        self.arm_pub.publish(arm_msg)
        
        # ハンド制御（R1ボタン）
        if len(msg.buttons) > 5 and msg.buttons[5] == 1:
            hand_msg = Float64MultiArray()
            hand_msg.data = [
                float(msg.axes[2] * 0.5) if len(msg.axes) > 2 else 0.0,
                float(msg.axes[5] * 0.5) if len(msg.axes) > 5 else 0.0
            ]
            # ハンド制御を送信
            self.hand_pub.publish(hand_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyToArmControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()