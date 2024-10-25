import cv2
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()

# Configure the camera resolution
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)

# Start the camera
picam2.start()

try:
    while True:
        # Capture frame-by-frame
        frame = picam2.capture_array()

        # Display the resulting frame
        cv2.imshow('Live Camera Feed', frame)

        # Exit the live feed when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Release the camera and close all OpenCV windows
    picam2.stop()
    cv2.destroyAllWindows()
