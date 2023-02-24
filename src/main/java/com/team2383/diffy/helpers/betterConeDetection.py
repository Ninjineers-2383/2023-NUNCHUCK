import numpy as np
import cv2
import logging
from networktables import NetworkTables

logging.basicConfig(level=logging.DEBUG)

NetworkTables.initialize()
SmartDashboard = NetworkTables.getTable("SmartDashboard")

# image = cv2.imread('/Users/john/FRC 2023/2023-Diffy-Swerve/src/main/java/com/team2383/diffy/helpers/SampleImages/img1.png')
cap = cv2.VideoCapture(0) # 0 is the default camera

while cap.isOpened():
    
    ret, frame = cap.read()
    
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    
    if frame is None:
        break
    
    original = frame.copy()
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(image, (5, 5), sigmaX=1.5, borderType=cv2.BORDER_DEFAULT)
    lower = np.array([24, 53, 30], dtype="uint8")
    upper = np.array([32, 255, 255], dtype="uint8")
    mask = cv2.inRange(blur, lower, upper)

    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] if len(contours) == 2 else contours[1]

    for contour in contours:
        
        approx = cv2.approxPolyDP(contour, 0.05 * cv2.arcLength(contour, True), True)
        kx = approx.ravel()[0]
        ky = approx.ravel()[1]
        
        if(len(approx) == 4):
            # x,y,w,h = cv2.boundingRect(contour)
            cv2.drawContours(original, [approx], 0, (36,255,12), 5)
            
            n = approx.ravel()
            i = 0
            
            for j in n :
                if(i % 2 == 0):
                    x = n[i]
                    y = n[i + 1]
        
                    # String containing the co-ordinates.
                    string = str(x) + " " + str(y) 
        
                    if(i == 0):
                        # text on topmost co-ordinate.
                        cv2.putText(original, "Arrow tip", (x, y),
                                        cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0)) 
                    else:
                        # text on remaining co-ordinates.
                        cv2.putText(original, string, (x, y), 
                                cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0)) 
                i = i + 1
            
            # if(w > 60 and h > 60):
            #     # Rectangle around cone
            #     cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 5)
            #     # Rectangle that indicates the center of the cone
            #     cv2.rectangle(original, (x + int(w/2) + 30, y + int(h/2) + 30), (x + int(w/2) - 30, y + int(h/2) - 30), (0,128,255), -1)
                
            #     # Distance indicator to determine how far the cone is from the camera
            #     distanceIndicator = h
            #     print("Distance: " + str(h))
                
            #     # SmartDashboard values
            #     SmartDashboard.putNumber("Cone Center X", str(x + int(w/2) - width/2))
            #     SmartDashboard.putNumber("Cone Center Y", str(y + int(h/2) - height/2))
            #     SmartDashboard.putNumber("Cone Distance Indicator", distanceIndicator)
    
    cv2.imshow("Stream", mask)

    if cv2.waitKey(0) and 0xFF == ord('w'):
        break

            
cap.release()
cv2.destroyAllWindows()