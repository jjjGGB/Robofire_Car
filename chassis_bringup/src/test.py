
import serial
import time

# 请根据实际情况修改端口号和波特率
port = '/dev/ttyACM0'
baud_rate = 115200

try:
    # 打开串口
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"成功打开串口: {ser.name}")
    while True:
            # 获取缓冲区里的字节数
            bytes_waiting = ser.in_waiting
            if bytes_waiting > 0:
                # 读取所有可用字节
                raw_data = ser.read(bytes_waiting)
                
                # 将字节流转换为十六进制字符串，中间加空格方便阅读
                hex_data = ' '.join([f"{b:02X}" for b in raw_data])
                print(f"收到 HEX: {hex_data}")
                
            time.sleep(0.01)

except serial.SerialException as e:
    print(f"串口错误: {e}")
    print("提示: 检查端口号是否正确，或者是否需要 sudo 权限 (将当前用户加入 dialout 组)")
except KeyboardInterrupt:
    print("\n退出读取")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()