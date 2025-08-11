#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
    print("=== Hello World Output ===")
    for _ in range(20):  # 10秒間読み取り
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(data)
        time.sleep(0.5)
    ser.close()
except Exception as e:
    print(f"Error: {e}")