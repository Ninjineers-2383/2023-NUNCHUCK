import cv2
import numpy as np
import glob

# cap = cv2.VideoCapture(0)
images = [cv2.imread(file) for file in glob.glob(
    "/Users/john/FRC 2023/2023-Diffy-Swerve/src/main/java/com/team2383/diffy/helpers/SampleImages/*.png")]

for image in images:
    # copy = image.copy()
    # ret, thresh = cv2.adaptiveThreshold(image, )
    # cv2.imshow('thresh', thresh)
    # cv2.waitKey(0)
    # if cv2.waitKey and 0xFF == ord('enter'):
    #    break
    img_copy = image.copy()
    gaussianFilter = cv2.GaussianBlur(img_copy, (7, 7), 0)

    # yellow filtering
    hsv = cv2.cvtColor(gaussianFilter, cv2.COLOR_BGR2HSV)
    lowerYellow = np.array([134, 170, 0])
    upperYellow = np.array([240, 226, 182])

    mask = cv2.inRange(hsv, lowerYellow, upperYellow)
    rgbFilter = cv2.bitwise_and(image, image, mask=mask)
    rgbFilter = cv2.normalize(src=rgbFilter, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # detecting cone
    grayscale = cv2.cvtColor(rgbFilter, cv2.COLOR_RGB2GRAY)
    coneDetect = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,1,100,minRadius=0,maxRadius=0)  # edge image, thresholding val, region size detecting peaks

    circles = np.uint16(np.around(coneDetect))
# Draw the circles
    for i in circles[0,:]:
    # draw the outer circle
        cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
        cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
    # print(grayscale.shape)
    # showing image
    cv2.imshow("edge rgb", image)

    if cv2.waitKey(1) and 0xFF == ord('q'):
        break


# while cap.isOpened():
#     original = image.copy()
#     image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
#     lower = np.array([22, 93, 0], dtype="uint8") # Lower HSV values
#     upper = np.array([45, 255, 255], dtype="uint8") # Higher HSV values
#     mask = cv2.inRange(image, lower, upper)

#     contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # Contours the shape
#     contours = contours[0] if len(contours) == 2 else contours[1]

#     for contour in contours:
#         # x,y,w,h = cv2.boundingRect(contour)
#         cv2.drawContours(original, [contour], -1, (36,255,12), 2) # Draws a contour around the shape
#         # cv2.rectangle(original, (x, y), (x + w, y + h), (36,255,12), 2) # Draws a rectangle around the shape

#     cv2.imshow('mask', mask)
#     cv2.imshow('original', original)
#     cv2.waitKey()

#     if cv2.waitKey and 0xFF ==ord('enter'):
#         break

# cap.release()
cv2.destroyAllWindows()