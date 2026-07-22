import serial
import time

PORT = 'COM21'
BAUD = 2000000
DEVICE_ID = 5

def make_request_packet(dev_id):
    # 恢复为“直接点名模式”：指定 ID，发送 CMD=1 请求
    header = 0x80 | ((dev_id & 0x1F) << 2) | 1
    return bytes([header, 0x00, 0x00, 0x00])

def decode_packet(packet):
    if len(packet) != 4 or not (packet[0] & 0x80):
        return None
    header7 = packet[0] & 0x7F
    r_id = header7 >> 2
    r_cmd = header7 & 0x03
    d0 = packet[1]
    d1 = packet[2]
    d2 = packet[3]
    data21 = d0 | (d1 << 7) | (d2 << 14)
    return r_id, r_cmd, data21

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    print(f"[OK] Opened port {PORT}, baud {BAUD}")
except Exception as e:
    print(f"[ERROR] Cannot open port: {e}")
    exit(1)

req_pkt = make_request_packet(DEVICE_ID)
print(f"[INFO] Requesting ID={DEVICE_ID} repeatedly... (Ctrl+C to quit)")

try:
    while True:
        ser.write(req_pkt)
        ser.flush()
        
        # 等待编码器回复
        time.sleep(0.02)
        resp = ser.read_all()
        if len(resp) > 0:
            print(f"[RAW RECV] {resp.hex().upper()}")
            
        found = False
        idx = 0
        while idx <= len(resp) - 4:
            if (resp[idx] & 0x80) != 0:
                pkt = resp[idx:idx+4]
                decoded = decode_packet(pkt)
                if decoded:
                    ret_id, ret_cmd, ret_data = decoded
                    
                    # 过滤掉我们自己发出去的请求包 (请求包的数据全是0)
                    if not (ret_id == DEVICE_ID and ret_cmd == 1 and ret_data == 0):
                        # 解析角度: 还原 15-bit 补码
                        raw15 = (ret_data >> 6) & 0x7FFF
                        if raw15 & 0x4000:
                            raw15_val = raw15 - 32768
                        else:
                            raw15_val = raw15
                            
                        degrees = (raw15_val * 360.0) / 32768.0
                        print(f"[RECV] Node ID: {ret_id} | Raw: 0x{ret_data:06X} | Angle: {degrees:>7.2f} deg")
                        found = True
                        break
            idx += 1
            
        if not found:
            if len(resp) > 0:
                print(f"[WARN] Unknown bytes received: {resp.hex()}")
            else:
                print(f"[TIMEOUT] No response. Check: 1. A/B wiring 2. Power 3. ID is {DEVICE_ID} 4. Flashed?")
                
        time.sleep(0.2)
except KeyboardInterrupt:
    print("测试结束。")
    ser.close()

