import cv2

cap = cv2.VideoCapture('/dev/video0')

if not cap.isOpened():
    print("Error: Could not open camera.")
else:
    ret, frame = cap.read()
    if ret:
        cv2.imwrite('test_frame.jpg', frame)
        print("Frame saved as 'test_frame.jpg'")
    else:
        print("Error: Could not read frame.")

cap.release()
