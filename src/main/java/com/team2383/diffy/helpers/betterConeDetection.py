import numpy as np
import cv2

image = cv2.imread('/Users/john/FRC 2023/2023-Diffy-Swerve/src/main/java/com/team2383/diffy/helpers/SampleImages/img1.png')
original = image.copy()
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
blur = cv2.GaussianBlur(image, (3, 3), sigmaX=1.5, borderType=cv2.BORDER_DEFAULT)
lower = np.array([24, 93, 20], dtype="uint8")
upper = np.array([43, 255, 255], dtype="uint8")
mask = cv2.inRange(blur, lower, upper)

cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = cnts[0] if len(cnts) == 2 else cnts[1]

for c in cnts:
    x,y,w,h = cv2.boundingRect(c)
    # cv2.drawContours(original, [c], -1, (36,255,12), 5)
    if(w > 50 and h > 50):
        cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 5)
        print("Width: " + str(w))
        print("Height: " + str(h))
        cv2.rectangle(original, (x + int(w/2) + 30, y + int(h/2) + 30), (x + int(w/2) - 30, y + int(h/2) - 30), (0,128,255), -1)

cv2.imshow('mask', mask)
cv2.imshow('Blurred', blur)
cv2.imshow('original', original)

if cv2.waitKey(0) & 0xFF == ord('enter'):
	cv2.destroyAllWindows()

# import cv2
# import numpy as np
# import glob

# # cap = cv2.VideoCapture(0)
# imgFilePath = "/Users/john/FRC 2023/2023-Diffy-Swerve/src/main/java/com/team2383/diffy/helpers/*.png"
# imgNames = glob.glob(imgFilePath)
# # img = cv2.imread(imgFilePath)

# lowerYellow = cv2.cvtColor(np.uint8([[[134, 170, 0]]]), cv2.COLOR_BGR2HSV)
# upperYellow = cv2.cvtColor(np.uint8([[[240, 226, 182]]]), cv2.COLOR_BGR2HSV)

# for file in imgNames:
    
#     print(file)

#     ret,frame = cv2.imread(file)
    
#     gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
#     mask = cv2.inRange(frame,lowerBound = lowerYellow, upperbBound = upperYellow)
#     mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
#     frame = frame & mask_rgb
#     cv2.imshow("Image",frame)

#     if cv2.waitKey and 0xFF == ord('enter'):
#        break