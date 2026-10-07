import cv2
from picamera2 import Picamera2

# Initialize Picamera2
picam2 = Picamera2()

# Configure the camera with preview settings
camera_config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (960, 720)})
picam2.configure(camera_config)

# Start the camera
picam2.start()

while True:
     frame = picam2.capture_array()
     frame = cv2.rotate(frame, cv2.ROTATE_180)
     frame=cv2.resize(frame,(960, 720))
     cv2.imshow('FRAME',frame)
     if cv2.waitKey(1) == ord('a'):
        print ("pressed a")
        frame=cv2.imwrite("/home/arshia/final_project/opencv-distance/FinalCode2/rf.png",frame)
        break
        
picam2.close()
cv2.destroyAllWindows()
