import cv2

# Initialize the camera
cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Grab an image from the camera
ret, frame = cap.read()

# Save opencv image
cv2.imwrite("image.jpg", frame)

# Release the camera
cap.release()
