"""读 TCRT5000 6路 ADC 原始值 — 辅助调阈值"""
import serial
import sys

port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'

ser = serial.Serial(port, 115200, timeout=1)

# 发 R 命令（\n 结尾，跟固件协议一致）
ser.write(b'R\n')

# 读返回
resp = ser.read(64)
print(resp.decode(errors='replace'))

ser.close()
