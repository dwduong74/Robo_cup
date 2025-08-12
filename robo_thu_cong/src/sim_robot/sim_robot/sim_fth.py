#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np
from std_msgs.msg import String
import json
import time

class Robot2DSim(Node):
    def __init__(self):
        super().__init__('sim_2d')
        self.sub = self.create_subscription(String, 'esp_vel', self.callback, 10)

        # Trạng thái robot
        self.x = 7.5
        self.y = -7.5
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.k = 0

        # Lưu quỹ đạo đã đi
        self.path_x = [self.x]
        self.path_y = [self.y]

        # Danh sách đạn: [(x, y, theta)]
        self.bullets = []

        # Thời gian hồi bắn
        self.last_shot_time = 0.0
        self.shot_cooldown = 1.0  # giây

    def callback(self, msg):
        try:
            data = json.loads(msg.data)  # parse JSON
            self.vy = float(data.get("vx", 0.0)) * 3
            self.vx = float(data.get("vy", 0.0)) * 3
            self.w  = - float(data.get("w", 0.0))
            self.k  = int(data.get("k", 0))

            # Nếu k == 80 và cooldown xong → bắn đạn
            now = time.time()
            if self.k == 80 and (now - self.last_shot_time) >= self.shot_cooldown:
                bullet_x = self.x + 0.5 * np.cos(self.theta)
                bullet_y = self.y + 0.5 * np.sin(self.theta)
                self.bullets.append([bullet_x, bullet_y, self.theta])
                self.last_shot_time = now
        except Exception as e:
            self.get_logger().error(f"Lỗi parse JSON: {e}")

    def update_state(self, dt=0.1):
        # Cập nhật robot
        dx = self.vx * dt
        dy = self.vy * dt
        dtheta = self.w * dt

        dx_world = dx * np.cos(self.theta) - dy * np.sin(self.theta)
        dy_world = dx * np.sin(self.theta) + dy * np.cos(self.theta)

        self.x += dx_world
        self.y += dy_world
        self.theta += dtheta

        # Lưu path
        self.path_x.append(self.x)
        self.path_y.append(self.y)

        # Cập nhật vị trí đạn
        bullet_speed = 5.0  # m/s
        for bullet in self.bullets:
            bullet[0] += bullet_speed * dt * np.cos(bullet[2])
            bullet[1] += bullet_speed * dt * np.sin(bullet[2])

        # Xóa đạn ra ngoài khung
        self.bullets = [b for b in self.bullets if abs(b[0]) <= 10 and abs(b[1]) <= 10]

def run_plot(node: Robot2DSim):
    fig, ax = plt.subplots()
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect('equal')
    plt.grid(True)
    plt.title("Robot 2D Simulator")

    # Robot và đường đi
    robot_shape, = ax.plot([], [], 'b-')
    path_line, = ax.plot([], [], 'r--', linewidth=1)
    bullet_dots, = ax.plot([], [], 'go', markersize=5)  # đạn màu xanh lá

    def update(frame):
        node.update_state()

        # Vẽ robot
        size = 1.0
        angle = node.theta
        cx, cy = node.x, node.y

        points = np.array([
            [size, 0],
            [-size/2, size/2],
            [-size/2, -size/2]
        ])

        R = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])

        rotated = (R @ points.T).T + np.array([cx, cy])
        robot_shape.set_data(
            [rotated[0][0], rotated[1][0], rotated[2][0], rotated[0][0]],
            [rotated[0][1], rotated[1][1], rotated[2][1], rotated[0][1]]
        )

        # Vẽ đường đi
        path_line.set_data(node.path_x, node.path_y)

        # Vẽ đạn
        if node.bullets:
            bullet_xs = [b[0] for b in node.bullets]
            bullet_ys = [b[1] for b in node.bullets]
            bullet_dots.set_data(bullet_xs, bullet_ys)
        else:
            bullet_dots.set_data([], [])

        return robot_shape, path_line, bullet_dots

    ani = FuncAnimation(fig, update, interval=10)
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = Robot2DSim()
    threading.Thread(target=run_plot, args=(node,), daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
