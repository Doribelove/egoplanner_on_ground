#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import math
import rospy
from geometry_msgs.msg import PoseStamped

# ================= 配置 =================
SERIAL_PORT = '/dev/ttyUSB1'  # 请确认端口
BAUD_RATE = 115200

# 全局变量存储原点
ORIGIN_LAT = None
ORIGIN_LON = None
# =======================================

def parse_nmea_lat_lon(line):
    """解析 GPS 数据"""
    try:
        parts = line.split(',')
        if not (line.startswith('$GNGGA') or line.startswith('$GPGGA')):
            return None
        if parts[2] == '' or parts[4] == '' or parts[6] == '0':
            return None

        # 纬度
        lat_raw = float(parts[2])
        lat = int(lat_raw / 100) + (lat_raw % 100) / 60.0
        if parts[3] == 'S': lat = -lat

        # 经度
        lon_raw = float(parts[4])
        lon = int(lon_raw / 100) + (lon_raw % 100) / 60.0
        if parts[5] == 'W': lon = -lon

        return lat, lon
    except Exception:
        return None

def get_gps_data():
    """读取一次串口数据"""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            start_t = time.time()
            # print("正在读取GPS...")
            while time.time() - start_t < 2:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                result = parse_nmea_lat_lon(line)
                if result: return result
            print("超时：未获取到定位")
            return None
    except Exception as e:
        print(f"串口错误: {e}")
        return None

def gps_to_xy(target_lat, target_lon):
    """
    计算 target 相对于 ORIGIN 的坐标
    """
    global ORIGIN_LAT, ORIGIN_LON
    
    # 1. 如果还没有原点，先尝试读取一次当前位置作为原点
    if ORIGIN_LAT is None:
        print("--> [系统提示] 正在获取当前位置作为地图原点 (0,0)...")
        gps = get_gps_data()
        if gps:
            ORIGIN_LAT, ORIGIN_LON = gps
            print(f"--> [原点已设定] Lat: {ORIGIN_LAT:.7f}, Lon: {ORIGIN_LON:.7f}")
        else:
            print("--> [错误] 无法设定原点，请确保 GPS 连接正常")
            return None, None

    # 2. 计算相对距离 (米)
    R = 6378137.0 # 地球半径
    d_lat = math.radians(target_lat - ORIGIN_LAT)
    d_lon = math.radians(target_lon - ORIGIN_LON)
    r_lat = math.radians(ORIGIN_LAT)

    x = R * d_lon * math.cos(r_lat) 
    y = R * d_lat

    return x, y

def publish_ros_goal(x, y):
    """发布目标点到 ROS"""
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera_init" 
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.w = 1.0
    
    time.sleep(0.5)
    pub.publish(msg)
    print(f"--> [ROS] 目标点已发送: X={x:.2f}, Y={y:.2f}")

def get_user_lat_lon():
    """从终端获取用户输入的经纬度"""
    print("\n请输入目标经纬度 (格式: 纬度 经度)")
    print("示例: 30.754544 103.923655")
    raw = input(">>> ").strip()
    
    try:
        # 兼容逗号或空格分隔
        raw = raw.replace(',', ' ')
        parts = raw.split()
        
        if len(parts) >= 2:
            lat = float(parts[0])
            lon = float(parts[1])
            return lat, lon
        else:
            print("[错误] 输入格式不对，至少需要两个数字")
            return None, None
    except ValueError:
        print("[错误] 包含非数字字符")
        return None, None

def main():
    rospy.init_node('gps_input_tool', anonymous=True)
    
    while not rospy.is_shutdown():
        print("\n=== GPS 终端输入工具 ===")
        print("1: 初始化原点 (读取当前车位设为 0,0)")
        print("2: 输入 GPS -> 计算局部坐标 (查看距离)")
        print("3: 输入 GPS -> 发送 ROS 目标点 (导航)")
        print("q: 退出")
        
        cmd = input("指令: ").strip()

        if cmd == '1':
            get_gps_data() # 触发一次读取
            # 强制重置
            ORIGIN_LAT = None
            # 再调用一次以触发设置逻辑
            gps_to_xy(0, 0) 
            print("原点已更新为当前位置。")

        elif cmd == '2':
            lat, lon = get_user_lat_lon()
            if lat is not None:
                x, y = gps_to_xy(lat, lon)
                if x is not None:
                    print("-" * 30)
                    print(f"目标 GPS: {lat}, {lon}")
                    print(f"相对原点: X(东)={x:.2f}m, Y(北)={y:.2f}m")
                    print("-" * 30)

        elif cmd == '3':
            lat, lon = get_user_lat_lon()
            if lat is not None:
                x, y = gps_to_xy(lat, lon)
                if x is not None:
                    print(f"准备前往: X={x:.2f}, Y={y:.2f}")
                    publish_ros_goal(x, y)

        elif cmd == 'q':
            break

if __name__ == '__main__':
    main()
