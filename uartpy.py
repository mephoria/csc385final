import serial
import json
import time
import threading
from datetime import datetime

class STM32CommandInterface:
    def __init__(self, port='COM4', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.running = True
        self.telemetry_data = {}
        self.imu_data = {}
        self.response_queue = []
        
        # Start background thread for receiving data
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
    def _receive_loop(self):
        """Background thread to continuously receive and parse data"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    self._parse_received_data(line)
            except Exception as e:
                print(f"Receive error: {e}")
            time.sleep(0.01)
    
    def _parse_received_data(self, line):
        """Parse different types of incoming data"""
        timestamp = datetime.now().isoformat()
        
        if line.startswith("TELEMETRY:"):
            try:
                json_data = line[10:]  # Remove "TELEMETRY:" prefix
                self.telemetry_data = json.loads(json_data)
                self.telemetry_data['received_at'] = timestamp
                print(f"[TELEMETRY] {self.telemetry_data}")
            except json.JSONDecodeError:
                print(f"Invalid telemetry JSON: {line}")
                
        elif line.startswith("IMU_DATA:"):
            try:
                json_data = line[9:]  # Remove "IMU_DATA:" prefix
                self.imu_data = json.loads(json_data)
                self.imu_data['received_at'] = timestamp
                print(f"[IMU] Tilt X:{self.imu_data['tilt_x']:.2f}° Y:{self.imu_data['tilt_y']:.2f}°")
            except json.JSONDecodeError:
                print(f"Invalid IMU JSON: {line}")
                
        elif line.startswith("STATUS:"):
            print(f"[STATUS] {line[7:]}")
            self.response_queue.append(line)
            
        elif line.startswith("ACK:"):
            print(f"[ACK] {line[4:]}")
            self.response_queue.append(line)
            
        elif line.startswith("ERROR:"):
            print(f"[ERROR] {line[6:]}")
            self.response_queue.append(line)
            
        elif line.startswith("TILT_X:"):
            # Legacy format support
            try:
                parts = line.split(',')
                tilt_x = float(parts[0].split(':')[1])
                tilt_y = float(parts[1].split(':')[1])
                self.imu_data = {
                    'tilt_x': tilt_x,
                    'tilt_y': tilt_y,
                    'received_at': timestamp
                }
                print(f"[IMU_LEGACY] X:{tilt_x:.2f}° Y:{tilt_y:.2f}°")
            except:
                pass
        else:
            print(f"[RAW] {line}")
    
    def send_command(self, command, wait_for_response=True, timeout=2.0):
        """Send command and optionally wait for response"""
        print(f"[SEND] {command}")
        self.ser.write(f"{command}\r\n".encode())
        
        if wait_for_response:
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.response_queue:
                    return self.response_queue.pop(0)
                time.sleep(0.1)
            return "TIMEOUT"
        return "SENT"
    
    # Command methods
    def get_status(self):
        """Get system status"""
        return self.send_command("GET_STATUS")
    
    def set_mode(self, mode):
        """Set system mode (0=IDLE, 1=IMU_STREAMING, 2=SERVO_CONTROL, etc.)"""
        return self.send_command(f"SET_MODE:{mode}")
    
    def calibrate_imu(self):
        """Calibrate IMU"""
        return self.send_command("CALIBRATE")
    
    def set_reporting_rate(self, rate_hz):
        """Set data reporting rate in Hz"""
        return self.send_command(f"SET_RATE:{rate_hz}")
    
    def get_imu_data(self):
        """Request immediate IMU data"""
        return self.send_command("GET_IMU")
    
    def get_telemetry(self):
        """Request telemetry data"""
        return self.send_command("GET_TELEMETRY")
    
    def set_led(self, state):
        """Control LED (ON/OFF)"""
        return self.send_command(f"SET_LED:{state}")
    
    def reset_system(self):
        """Reset the STM32"""
        return self.send_command("RESET", wait_for_response=False)
    
    def get_latest_imu(self):
        """Get the most recent IMU data"""
        return self.imu_data.copy() if self.imu_data else None
    
    def get_latest_telemetry(self):
        """Get the most recent telemetry data"""
        return self.telemetry_data.copy() if self.telemetry_data else None
    
    def close(self):
        """Close the connection"""
        self.running = False
        self.receive_thread.join(timeout=1)
        self.ser.close()

# Example usage and testing
def main():
    print("STM32 Command Interface Test")
    
    try:
        # Initialize connection
        stm32 = STM32CommandInterface('COM4', 115200)
        time.sleep(1)  # Allow connection to establish
        
        # Test commands
        print("\n=== Testing Commands ===")
        
        # Get initial status
        response = stm32.get_status()
        print(f"Status: {response}")
        
        # Set LED on
        response = stm32.set_led("ON")
        print(f"LED ON: {response}")
        
        # Get telemetry
        response = stm32.get_telemetry()
        print(f"Telemetry request: {response}")
        
        # Set IMU streaming mode
        response = stm32.set_mode(1)  # IMU_STREAMING
        print(f"Set streaming mode: {response}")
        
        # Monitor data for 10 seconds
        print("\n=== Monitoring Data (10 seconds) ===")
        start_time = time.time()
        
        while time.time() - start_time < 10:
            # Get latest data without sending commands
            imu = stm32.get_latest_imu()
            telemetry = stm32.get_latest_telemetry()
            
            if imu:
                print(f"Latest IMU: X={imu.get('tilt_x', 0):.2f}° Y={imu.get('tilt_y', 0):.2f}°")
            
            if telemetry:
                print(f"System uptime: {telemetry.get('uptime', 0)}ms, Mode: {telemetry.get('mode', 0)}")
            
            time.sleep(1)
        
        # Turn off streaming
        response = stm32.set_mode(0)  # IDLE
        print(f"Set idle mode: {response}")
        
        # Turn LED off
        response = stm32.set_led("OFF")
        print(f"LED OFF: {response}")
        
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        try:
            stm32.close()
        except:
            pass

if __name__ == "__main__":
    main()