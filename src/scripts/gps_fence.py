#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import time
import math
import sys

# ================= 配置区域 =================
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 115200

# 围栏半径 (米)
FENCE_RADIUS = 6.0 

# 围栏中心点 (运行功能1后会自动填入，或手动修改这里)
CENTER_LAT = None
CENTER_LON = None
# ===========================================

def parse_gps(line):
    """简单的字符串解析，提取经纬度"""
    try:
        parts = line.split(',')
        # 必须是 GNGGA 且包含数据
        if not (line.startswith('$GNGGA') or line.startswith('$GPGGA')): return None
        if parts[2] == '' or parts[4] == '' or parts[6] == '0': return None

        # 格式: ddmm.mmmm -> 度.度度度度
        # 纬度
        lat_raw = float(parts[2])
        lat = int(lat_raw/100) + (lat_raw % 100)/60.0
        if parts[3] == 'S': lat = -lat
        
        # 经度
        lon_raw = float(parts[4])
        lon = int(lon_raw/100) + (lon_raw % 100)/60.0
        if parts[5] == 'W': lon = -lon
        
        return lat, lon
    except:
        return None

def get_current_gps():
    """读取串口获取一次坐标"""
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            t_start = time.time()
            while time.time() - t_start < 2: # 2秒尝试
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                res = parse_gps(line)
                if res: return res
    except Exception as e:
        print(f"串口错误: {e}")
    return None

def calc_flat_distance(lat1, lon1, lat2, lon2):
    """
    【极简平面距离计算】
    忽略地球曲率，只适合短距离(如几百米内)
    """
    # 1度纬度 约为 111320 米
    METERS_PER_DEGREE = 111320.0
    
    # 纬度差直接转米
    delta_lat_m = (lat1 - lat2) * METERS_PER_DEGREE
    
    # 经度差转米 (需要乘 cosine修正)
    # 取平均纬度做修正系数
    avg_lat_rad = math.radians((lat1 + lat2) / 2.0)
    delta_lon_m = (lon1 - lon2) * METERS_PER_DEGREE * math.cos(avg_lat_rad)
    
    # 勾股定理
    dist = math.sqrt(delta_lat_m**2 + delta_lon_m**2)
    return dist

def set_center():
    """功能1：设置当前点为圆心"""
    global CENTER_LAT, CENTER_LON
    print("正在定位中...")
    gps = get_current_gps()
    if gps:
        CENTER_LAT, CENTER_LON = gps
        print(f"\n[OK] 圆心已设定!")
        print(f"     Lat: {CENTER_LAT:.8f}")
        print(f"     Lon: {CENTER_LON:.8f}")
        print("(如果不重启脚本，该设置一直有效)\n")
    else:
        print("[Error] 无法获取GPS信号")

def monitor():
    """功能2：每秒检测距离"""
    if CENTER_LAT is None:
        print("\n[Error] 请先执行功能 1 设置圆心！")
        return

    print(f"\n>>> 开始监控 (围栏半径: {FENCE_RADIUS}米) <<<")
    print("按 Ctrl+C 退出...")
    
    try:
        while True:
            gps = get_current_gps()
            if gps:
                curr_lat, curr_lon = gps
                # 计算距离
                dist = calc_flat_distance(curr_lat, curr_lon, CENTER_LAT, CENTER_LON)
                
                # 状态判断
                if dist <= FENCE_RADIUS:
                    # 安全 (绿色文字)
                    msg = f"\033[92m[安全] 距离: {dist:.2f}m\033[0m"
                else:
                    # 越界 (红色文字 + 警告)
                    msg = f"\033[91m[警告] 距离: {dist:.2f}m -> 已越界!!!\033[0m"
                
                # 动态刷新打印
                sys.stdout.write(f"\r{time.strftime('%H:%M:%S')} | {msg}      ")
                sys.stdout.flush()
            else:
                sys.stdout.write("\r信号丢失...      ")
                sys.stdout.flush()
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n监控停止。")

if __name__ == '__main__':
    while True:
        print("\n=== 简易电子围栏 ===")
        print("1. 设当前点为圆心")
        print("2. 开始监测 (每秒)")
        print("q. 退出")
        cmd = input("指令: ").strip()
        
        if cmd == '1': set_center()
        elif cmd == '2': monitor()
        elif cmd == 'q': break
