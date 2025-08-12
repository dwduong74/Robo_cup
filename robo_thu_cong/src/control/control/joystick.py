#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import pygame

class XboxControllerNode(Node):
    def __init__(self):
        super().__init__('xbox')

        # ROS publisher
        self.publisher = self.create_publisher(String, 'esp_vel', 10)

        # Tốc độ tối đa
        self.max_vx = 0.4
        self.max_vy = 0.4
        self.max_w  = .5
        self.max_x  = 200
        self.max_y  = 230
        self.max_k  = 80
        self.y_state = 0  # Trạng thái x, toggle giữa 0 và max_x
        # Khởi tạo pygame joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error(" Không tìm thấy tay cầm Xbox!")
            exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f" Xbox controller: {self.joystick.get_name()} đã sẵn sàng")

        # Gọi vòng lặp đọc tay cầm
        self.timer = self.create_timer(0.05, self.read_controller)  # 20Hz

    def read_controller(self):
        pygame.event.pump()  # cập nhật trạng thái controller

        # Đọc stick
        if abs(self.joystick.get_axis(0)) > 0.1 or abs(self.joystick.get_axis(1)) > 0.1:
            vx_axis = self.joystick.get_axis(0)
            vy_axis = self.joystick.get_axis(1)
            if abs(vx_axis) >abs(vy_axis):
                vx = self.max_vx * (vx_axis)
                vy = 0.0
            else:
                vy = self.max_vy * vy_axis
                vx = 0.0
        elif abs(self.joystick.get_axis(4)) > 0.1 or abs(self.joystick.get_axis(3)) > 0.5:
            vx_axis = self.joystick.get_axis(3)
            vy_axis = self.joystick.get_axis(4)
            if abs(vx_axis) > abs(vy_axis):
                vx = self.max_vx * (-vx_axis)
                vy = 0.0
            else:
                vx = 0.0
                vy = self.max_vy * -vy_axis
        else:
            vx = 0.0
            vy = 0.0

        # Đọc nút D-pad
        hat = self.joystick.get_hat(0)
        if hat != (0, 0):
            if hat == (0, 1):
                vy = 0.2
            elif hat == (1, 0):
                vx = -0.2
            elif hat == (0, -1):
                vy = -0.2
            elif hat == (-1, 0):
                vx = 0.2
        # Nút quay w: B (1), X (2)
        w = 0.0
        if self.joystick.get_button(1):  # B
            w = self.max_w
        elif self.joystick.get_button(2):  # X
            w = -self.max_w

        # Nút x: LB (4), y: RB (5), k: BACK (6)
        # Toggle nút LB (button 4)
        lb = self.joystick.get_button(6)
        if lb and not self.prev_lb_state:  # chỉ toggle khi nhấn xuống
            if self.y_state == self.max_y:
                self.y_state = 0
            else:
                self.y_state = self.max_y
        self.prev_lb_state = lb  # cập nhật trạng thái

        y = self.y_state

        x = self.max_x if self.joystick.get_button(4) else 0
        k = self.max_k if self.joystick.get_button(5) else 120
        if self.joystick.get_button(3) :
            w = 2.0
        if self.joystick.get_button(0) :
            w = -2.0    
        # Gửi message
        vx = round(vx, 2)
        vy = round(vy, 2)
        doc = {"vx": vx, "vy": vy, "w": w, "x": x, "y": y, "k": k}
        msg_out = String()
        msg_out.data = json.dumps(doc)
        self.publisher.publish(msg_out)

        self.get_logger().info(f" {msg_out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = XboxControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
