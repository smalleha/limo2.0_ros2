import serial
import time

def read_serial_hex_to_txt(port="/dev/ttyTHS1", baudrate=115200, output_file="output_hex.txt", duration=10):
    """
    读取串口的十六进制数据并保存为txt文件

    参数:
    - port: 串口号（如 /dev/ttyUSB0, /dev/ttyACM0）
    - baudrate: 波特率（如 9600, 115200）
    - output_file: 输出的文件名
    - duration: 读取时长（单位：秒）
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print(f"串口 {port} 已打开，开始读取十六进制数据...")

        with open(output_file, "w") as f:
            start_time = time.time()
            while time.time() - start_time < duration:
                if ser.in_waiting:
                    raw_bytes = ser.read(ser.in_waiting)
                    hex_str = raw_bytes.hex().upper()
                    # 格式化成 2位一组的 hex，例如 "01 02 03"
                    formatted_hex = ' '.join(hex_str[i:i+2] for i in range(0, len(hex_str), 2))
                    print(f"读取到：{formatted_hex}")
                    f.write(formatted_hex + "\n")

        ser.close()
        print(f"读取完成，十六进制数据已保存至 {output_file}")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("用户中断，退出。")
    except Exception as e:
        print(f"发生异常: {e}")

# 示例调用
if __name__ == "__main__":
    read_serial_hex_to_txt("/dev/ttyTHS1", 460800, "serial_hex_data.txt", duration=30)
