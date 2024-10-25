import smbus
import time

# MPU-6050 registers and addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# Initialize the I2C bus
bus = smbus.SMBus(1)

# Wake up the MPU-6050 since it starts in sleep mode
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# Function to read raw data from the sensor
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    
    if value > 32768:
        value -= 65536
    return value

# Function to calculate acceleration in g
def get_acceleration():
    accel_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    accel_y = read_raw_data(ACCEL_YOUT_H) / 16384.0
    accel_z = read_raw_data(ACCEL_ZOUT_H) / 16384.0
    return accel_x, accel_y, accel_z

# Set the threshold for detecting a bump (change this value based on your testing)
THRESHOLD_G = 1.15

try:
    while True:
        # Get the acceleration values from the sensor
        accel_x, accel_y, accel_z = get_acceleration()
        
        # Calculate the magnitude of the acceleration vector
        accel_total = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
        
        # Print the values (for testing and observation)
        print(f"X: {accel_x:.2f}, Y: {accel_y:.2f}, Z: {accel_z:.2f}, Total: {accel_total:.2f}")
        
        # Check if the total acceleration exceeds the threshold
        if accel_total > THRESHOLD_G:
            print("ALERT: Bump detected! Acceleration exceeded threshold.")
        
        # Wait for a short time before reading again
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program terminated.")
