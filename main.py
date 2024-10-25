import smbus
import time
import RPi.GPIO as GPIO
import subprocess
import random
from gps import gps, WATCH_ENABLE
from datetime import datetime

# Initialize GPS (if you're using it)
gpsd = gps(mode=WATCH_ENABLE)

# Set up GPIO pins for IR sensors
LEFT_IR_SENSOR_PIN = 17
RIGHT_IR_SENSOR_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_IR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(RIGHT_IR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# MPU-6050 registers and addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# Initialize the I2C bus
bus = smbus.SMBus(1)

# MPU6050 setup: Wake up the MPU-6050 from sleep mode
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

# Function to capture image using libcamera-still with a dynamic name based on the current time
def capture_image_with_time():
    """Capture an image using libcamera-still with the current timestamp in the file name."""
    current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    image_path = f"/home/capstone/detection_{current_time}.jpg"
    try:
        subprocess.run(["libcamera-still", "-o", image_path, "-t", "1"], check=True)
        print(f"Image captured and saved to {image_path}")
    except subprocess.CalledProcessError as e:
        print(f"Failed to capture image: {e}")

# Set threshold for vibration detection
THRESHOLD_G = 1.15

# Function to get current date and time as a string
def get_current_time():
    """Returns the current date and time as a formatted string."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")

# Function to generate random GPS coordinates within a specified range
def generate_random_gps():
    """Generate random GPS coordinates."""
    lat = random.uniform(-90.0, 90.0)  # Latitude range
    lon = random.uniform(-180.0, 180.0)  # Longitude range
    return lat, lon

# Countdown function before taking the picture
def countdown(seconds):
    """Countdown before taking a picture."""
    for i in range(seconds, 0, -1):
        print(f"Taking picture in {i} seconds...")
        time.sleep(1)

def monitor_railway():
    """Monitors the railway track with IR sensors and MPU, and captures images if anomalies are detected."""
    try:
        while True:
            # IR sensor detection (reversed logic)
            left_rail_detected = GPIO.input(LEFT_IR_SENSOR_PIN)
            right_rail_detected = GPIO.input(RIGHT_IR_SENSOR_PIN)

            if left_rail_detected == 0 and right_rail_detected == 0:
                print("Railway Track detected on both sides")
            elif left_rail_detected == 1 and right_rail_detected == 0:
                current_time = get_current_time()
                lat, lon = generate_random_gps()
                print(f"No Railway Track on Left Side. Time of obstruction: {current_time}. Obstruction detected at coordinates: ({lat:.6f}, {lon:.6f})")
                print("Taking picture in 10 seconds...")
                countdown(10)
                capture_image_with_time()  # Save image with timestamped filename
                time.sleep(15)  # Cooldown for 15 seconds before restarting
                continue
            elif left_rail_detected == 0 and right_rail_detected == 1:
                current_time = get_current_time()
                lat, lon = generate_random_gps()
                print(f"No Railway Track on Right Side. Time of obstruction: {current_time}. Obstruction detected at coordinates: ({lat:.6f}, {lon:.6f})")
                print("Taking picture in 10 seconds...")
                countdown(10)
                capture_image_with_time()  # Save image with timestamped filename
                time.sleep(15)  # Cooldown for 15 seconds before restarting
                continue
            elif left_rail_detected == 1 and right_rail_detected == 1:
                current_time = get_current_time()
                lat, lon = generate_random_gps()
                print(f"No Railway Track detected on both sides. Time of obstruction: {current_time}. Obstruction detected at coordinates: ({lat:.6f}, {lon:.6f})")
                print("Taking picture in 10 seconds...")
                countdown(10)
                capture_image_with_time()  # Save image with timestamped filename
                time.sleep(15)  # Cooldown for 15 seconds before restarting
                continue

            # MPU-6050 detection
            accel_x, accel_y, accel_z = get_acceleration()
            accel_total = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5

            if accel_total > THRESHOLD_G:
                current_time = get_current_time()
                lat, lon = generate_random_gps()
                print(f"Halt! Threshold detected due to high vibration. Detected g-force: {accel_total:.2f}g. Time of detection: {current_time}. Coordinates: ({lat:.6f}, {lon:.6f})")
                print("Taking picture in 10 seconds...")
                countdown(10)
                capture_image_with_time()  # Save image with timestamped filename
                time.sleep(15)  # Cooldown for 15 seconds before restarting
                continue
            else:
                print(f"Railway track detected and normal threshold running. Current g-force: {accel_total:.2f}g")

            # Delay before next check
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Program terminated.")

    finally:
        # Clean up GPIO resources
        GPIO.cleanup()

# Start monitoring the railway
monitor_railway()
