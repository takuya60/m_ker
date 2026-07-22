import serial
import time

PORT = 'COM21'
BAUD = 2000000

def make_request_packet(dev_id):
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
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    print(f"[OK] Opened port {PORT}, scanning IDs 1 to 16...")
except Exception as e:
    print(f"[ERROR] Cannot open port: {e}")
    exit(1)

for test_id in range(1, 17):
    req_pkt = make_request_packet(test_id)
    ser.write(req_pkt)
    ser.flush()
    time.sleep(0.01)
    
    resp = ser.read_all()
    found = False
    
    idx = 0
    while idx <= len(resp) - 4:
        if (resp[idx] & 0x80) != 0:
            pkt = resp[idx:idx+4]
            decoded = decode_packet(pkt)
            if decoded:
                ret_id, ret_cmd, ret_data = decoded
                if not (ret_id == test_id and ret_cmd == 1 and ret_data == 0):
                    print(f"!!! SUCCESS !!! Board responded to ID = {ret_id}")
                    found = True
                    break
        idx += 1
    
ser.close()
print("Scan complete.")
