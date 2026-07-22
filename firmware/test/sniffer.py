import serial
import time

PORT = 'COM21'
BAUD = 2000000

try:
    ser = serial.Serial(PORT, BAUD, timeout=0.01)
    print(f"[SNIFFER] Listening on {PORT} at {BAUD} baud...")
except Exception as e:
    print(f"[ERROR] Cannot open port: {e}")
    exit(1)

print("Make sure A is connected to A, B to B across PC, M5, and Encoder.")
print("Waiting for data on the RS485 bus (Ctrl+C to quit)...")

try:
    while True:
        data = ser.read(1024)
        if data:
            hex_str = " ".join([f"{b:02X}" for b in data])
            print(f"[BUS DATA] {hex_str}")
except KeyboardInterrupt:
    print("\nSniffer stopped.")
    ser.close()
