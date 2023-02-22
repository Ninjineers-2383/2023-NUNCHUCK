import cv2
import numpy as np
import glob

# cap = cv2.VideoCapture(0)
imgFilePath = "SampleImages/*.png"
imgNames = glob.glob(imgFilePath)


lowerYellow = cv2.cvtColor(np.uint8([[[134, 170, 0]]]), cv2.COLOR_BGR2HSV)
upperYellow = cv2.cvtColor(np.uint8([[[240, 226, 182]]]), cv2.COLOR_BGR2HSV)

for file in imgNames:
    
    print(file)

    ret,frame = cv2.imread(file, 0)
    
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(frame,lowerBound = lowerYellow, upperbBound = upperYellow)
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    frame = frame & mask_rgb
    cv2.imshow("Image",frame)

    cv2.waitKey(0)
    if cv2.waitKey and 0xFF == ord('enter'):
       break