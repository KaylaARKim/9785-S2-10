import smbus
import RPi.GPIO as GPIO
import serial
import time
import datetime
from w1thermsensor import W1ThermSensor
from picamera import PiCamera

# Initialize the sensors

# MPU-6050 Setup
MPU_ADDRESS_1 = 0x68  # I2C address of the first MPU-6050
MPU_ADDRESS_2 = 0x69  # I2C address of the second MPU-6050
bus = smbus.SMBus(1)
bus.write_byte_data(MPU_ADDRESS_1, 0x6B, 0)  # Wake up MPU-6050 #1
bus.write_byte_data(MPU_ADDRESS_2, 0x6B, 0)  # Wake up MPU-6050 #2

# HC-SR04 Setup (for 4 sensors)
TRIG = [23, 24, 25, 26]  # GPIO pins for Trigger for 4 sensors
ECHO = [27, 28, 29, 30]  # GPIO pins for Echo for 4 sensors
GPIO.setmode(GPIO.BCM)
for i in range(4):
    GPIO.setup(TRIG[i], GPIO.OUT)
    GPIO.setup(ECHO[i], GPIO.IN)

# Neo-6M GPS Setup
gps_serial = serial.Serial('/dev/ttyAMA0', baudrate=9600, timeout=1)

# DS18B20 Setup (for 2 sensors)
temperature_sensors = W1ThermSensor.get_available_sensors()

# Camera Setup (for 4 cameras)
cameras = [PiCamera(camera_num=i) for i in range(4)]

# Function to read MPU-6050 data for both sensors
def read_mpu(mpu_address):
    acc_x = bus.read_word_data(mpu_address, 0x3B)
    acc_y = bus.read_word_data(mpu_address, 0x3D)
    acc_z = bus.read_word_data(mpu_address, 0x3F)
    gyro_x = bus.read_word_data(mpu_address, 0x43)
    gyro_y = bus.read_word_data(mpu_address, 0x45)
    gyro_z = bus.read_word_data(mpu_address, 0x47)
    return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

# Function to measure distance using HC-SR04 for all sensors
def measure_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

# Function to get GPS coordinates
def get_gps():
    data = gps_serial.readline().decode('ascii', errors='replace')
    if "$GPGGA" in data:
        parts = data.split(',')
        if parts[2] and parts[4]:
            latitude = float(parts[2]) / 100
            longitude = float(parts[4]) / 100
            return latitude, longitude
    return None, None

# Function to get temperature from multiple sensors
def get_temperatures():
    temperatures = []
    for sensor in temperature_sensors:
        temperature = sensor.get_temperature()
        temperatures.append(temperature)
    return temperatures

# Function to capture images from all cameras
def capture_images():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    for i, camera in enumerate(cameras):
        image_path = f'/home/pi/fishplate_image_camera_{i}_{timestamp}.jpg'
        camera.capture(image_path)
        print(f"Captured image from camera {i}: {image_path}")

# Main loop to gather data and check conditions
def main():
    while True:
        # Read data from both MPU-6050 sensors
        acc1 = read_mpu(MPU_ADDRESS_1)
        acc2 = read_mpu(MPU_ADDRESS_2)
        
        # Measure distances from all HC-SR04 sensors
        distances = []
        for i in range(4):
            distance = measure_distance(TRIG[i], ECHO[i])
            distances.append(distance)

        # Get temperatures from both DS18B20 sensors
        temperatures = get_temperatures()

        # Get GPS coordinates
        latitude, longitude = get_gps()

        # Example thresholds (these should be adjusted based on real data)
        if any(abs(val) > 2000 for val in acc1) or any(abs(val) > 2000 for val in acc2):
            print("Alert: High vibration detected")
            capture_images()  # Capture images if vibration is high
        if any(dist < 5 for dist in distances):  # Example threshold for deformation
            print("Alert: Fishplate deformation detected")
            capture_images()  # Capture images if deformation is detected
        if any(temp > 40 for temp in temperatures):  # Example threshold for temperature
            print("Alert: High temperature detected")
            capture_images()  # Capture images if temperature is high

        if latitude and longitude:
            print(f"Location: {latitude}, {longitude}")
        
        time.sleep(2)  # Delay for 2 seconds

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program stopped by user")
