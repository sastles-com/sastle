#!/usr/bin/env python3
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Serial connection established. Monitoring output...")
    print("=" * 60)
    
    while True:
        line = ser.readline().decode('utf-8', errors='replace').strip()
        if line:
            print(line)
            
except KeyboardInterrupt:
    print("\nMonitoring stopped by user")
except serial.SerialException as e:
    print(f"Serial error: {e}")
finally:
    if 'ser' in locals():
        ser.close()