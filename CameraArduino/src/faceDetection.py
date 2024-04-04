import cv2
import serial
import numpy as np

# Initialize the serial connections
ser = serial.Serial('/dev/cu.usbmodem1101', 921600, timeout=1) # Read from nano
ser_write = serial.Serial('/dev/cu.usbmodem103', 921600, timeout=1) #  Write to ST

# Load pre-trained face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

def read_image_from_serial(ser):
    start_marker = b'\xff\xd8'  # JPEG start
    end_marker = b'\xff\xd9'    # JPEG end
    data = ser.read_until(start_marker)
    image_data = start_marker + ser.read_until(end_marker)
    return image_data

try:
    while True:
        img_data = read_image_from_serial(ser)
        image = np.asarray(bytearray(img_data), dtype="uint8")
        frame = cv2.imdecode(image, cv2.IMREAD_COLOR)

        if frame is not None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.3, 4)

            # Sort faces based on face area (w*h) - largest area first
            faces = sorted(faces, key=lambda face: face[2] * face[3], reverse=True)
            
            if faces:
                # Focus only on the largest face
                x, y, w, h = faces[0]
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
                print(f"Face detected at X:{x} Y:{y} W:{w} H:{h}")

                # Send the largest face's coordinates
                ser_write.write(f"{x},{y},{w},{h}\n".encode())

            cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    ser.close()
    cv2.destroyAllWindows()
