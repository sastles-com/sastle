#!/usr/bin/env python3
import serial
import time
import sys

try:
    # シリアルポートを開く
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("シリアルポート接続成功")
    print("データを読み取り中... (Ctrl+Cで終了)")
    
    # 10秒間データを読み取り
    start_time = time.time()
    while time.time() - start_time < 10:
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').strip()
                if data:
                    print(f"受信: {data}")
            except UnicodeDecodeError:
                # バイナリデータの場合
                raw_data = ser.readline()
                print(f"受信(hex): {raw_data.hex()}")
        time.sleep(0.1)
        
except serial.SerialException as e:
    print(f"シリアルポートエラー: {e}")
except KeyboardInterrupt:
    print("終了します")
except Exception as e:
    print(f"エラー: {e}")
finally:
    try:
        ser.close()
    except:
        pass