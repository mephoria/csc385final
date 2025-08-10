import serial
import time

print("Testing serial connection...")

try:
    # Test COM4 specifically
    ser = serial.Serial('COM4', 115200, timeout=2)
    print("✓ Connected to COM4")
    
    print("Reading data from STM32...")
    print("(Move your IMU board to see data changes)")
    print("Press Ctrl+C to stop")
    
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"STM32: {line}")
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("\nStopped by user")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
    print("\nTroubleshooting:")
    print("1. Make sure STM32 is connected to COM4")
    print("2. Check if another program is using COM4")
    print("3. Try unplugging and reconnecting the STM32")
    print("4. Check Device Manager to confirm COM4 is correct")