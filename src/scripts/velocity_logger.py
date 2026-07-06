#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
import time
import csv

class VelocityPlotter:
    def __init__(self, topic_name='/cmd_vel'):
        self.topic_name = topic_name
        self.times = []
        self.linear_x = []
        self.angular_z = []
        self.start_time = None
        self.is_running = True

        # 初始化节点
        rospy.init_node('velocity_plotter', anonymous=True)
        
        # 订阅话题
        rospy.Subscriber(self.topic_name, Twist, self.callback)
        print(f"\n>>> 开始监听话题: {self.topic_name}")
        print(">>> 正在记录数据... (按 Ctrl+C 结束并绘图)")

    def callback(self, msg):
        if not self.is_running:
            return

        # 注意：使用 rospy.Time.now().to_sec() 通常比 rospy.get_time() 更稳定
        current_time = rospy.Time.now().to_sec()
        
        if self.start_time is None:
            self.start_time = current_time
        
        # 记录相对时间 (从0秒开始)
        dt = current_time - self.start_time
        
        self.times.append(dt)
        self.linear_x.append(msg.linear.x)
        self.angular_z.append(msg.angular.z)

    def save_and_plot(self):
        self.is_running = False
        if not self.times:
            print("\n[警告] 未接收到任何数据！")
            return

        print(f"\n\n>>> 停止记录，共收集 {len(self.times)} 帧数据。")
        print(">>> 正在绘制曲线...")

        # 1. 绘制曲线
        plt.figure(figsize=(10, 6))
        
        # 线速度子图
        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.linear_x, 'b-', label='Linear X (m/s)', linewidth=1.5)
        plt.title(f'Velocity Curve ({self.topic_name})')
        plt.ylabel('Linear Speed (m/s)')
        plt.grid(True)
        plt.legend()

        # 角速度子图
        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.angular_z, 'r-', label='Angular Z (rad/s)', linewidth=1.5)
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Speed (rad/s)')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()

        # 2. 保存图片
        img_filename = 'velocity_curve.png'
        try:
            plt.savefig(img_filename)
            print(f"[OK] 图片已保存: {img_filename}")
        except Exception as e:
            print(f"[Error] 图片保存失败: {e}")

        # 3. 保存 CSV 数据 (可选)
        csv_filename = 'velocity_data.csv'
        try:
            with open(csv_filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Time', 'Linear_X', 'Angular_Z'])
                for t, lx, az in zip(self.times, self.linear_x, self.angular_z):
                    writer.writerow([t, lx, az])
            print(f"[OK] 数据已保存: {csv_filename}")
        except Exception as e:
            print(f"[Error] CSV保存失败: {e}")

        # 显示图片 (这会阻塞程序，直到关闭窗口)
        print(">>> 显示图表，关闭窗口后退出程序。")
        plt.show()

if __name__ == '__main__':
    # 实例化类
    plotter = VelocityPlotter(topic_name='/cmd_vel')
    
    # 核心修改：移除 signal 模块，完全依赖 rospy 的机制
    try:
        # 主循环
        rate = rospy.Rate(10) # 10Hz 检查频率
        while not rospy.is_shutdown():
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        # 这里捕获标准的 Python Ctrl+C，双重保险
        pass
    finally:
        # 无论如何退出（ROS关闭或强制中断），都会执行绘图
        plotter.save_and_plot()
