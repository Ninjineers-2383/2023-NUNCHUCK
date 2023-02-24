import cv2
import numpy as np
import glob

cap = cv2.VideoCapture(0)

while cap.isOpened():
    
    ret, frame = cap.read()
    
    if frame is None:
        break
    
    cv2.imshow("Stream", frame)
    
    if cv2.waitKey(0) and 0xFF == ord('q'):
        break

            
cap.release()
cv2.destroyAllWindows()
