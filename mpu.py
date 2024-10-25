import RPi.GPIO as GPIO
import time
import subprocess
from gps import gps, WATCH_ENABLE

# Initialize GPS
gpsd = gps(mode=WATCH_ENABLE)

# Set up GPIO pins for IR sensors
LEFT_IR_SENSOR_PIN = 17
RIGHT_IR_SENSOR_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_IR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(RIGHT_IR_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# File paths to save images
detected_image_path = "/home/capstone/detected_image.jpg"
no_object_image_path = "/home/capstone/no_object_image.jpg"

def capture_image(image_path):
    """Capture an image using libcamera-still."""
    try:
        subprocess.run(["libcamera-still", "-o", image_path, "--nopreview"], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error capturing image: {e}")

def get_gps_coordinates():
    """Fetch GPS coordinates."""
    gpsd.next()  # This blocks until there's a new reading
    if gpsd.fix.mode >= 2 and gpsd.fix.latitude != 0 and gpsd.fix.longitude != 0:
        return (gpsd.fix.latitude, gpsd.fix.longitude)
    else:
        print("No GPS fix yet. Waiting for signal...")
        return None

try:
    rail_detected = False  # Track if rail is detected

    while True:
        left_detected = GPIO.input(LEFT_IR_SENSOR_PIN)
        right_detected = GPIO.input(RIGHT_IR_SENSOR_PIN)

        if left_detected and right_detected:
            print("Railway NOT detected on both left and right!")
            print("Taking picture...")
            capture_image(detected_image_path)
            gps_coords = get_gps_coordinates()
            if gps_coords:
                print(f"GPS coordinates: {gps_coords}")
            rail_detected = False  # Reset rail detected state

        elif left_detected:
            print("Railway NOT detected on the left!")
s            print("Taking picture...")
            capture_image(detected_image_path)
            gps_coords = get_gps_coordinates()
            rail_detected = False  # Reset rail detected state

        elif right_detected:
            print("Railway NOT detected on the right!")
            print("Taking picture...")
            capture_image(detected_image_path)
            gps_coords = get_gps_coordinates()
            rail_detected = False  # Reset rail detected state

        else:
            if not rail_detected:  # Only print if we weren't already detecting rail
                print("Railway detected.")
                gps_coords = get_gps_coordinates()
                if gps_coords:
                    print(f"GPS coordinates: {gps_coords}")
                rail_detected = True  # Set rail detected state
            else:
                # If rail is still detected, keep outputting the message
                print("Railway still detected.")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    GPIO.cleanup()
