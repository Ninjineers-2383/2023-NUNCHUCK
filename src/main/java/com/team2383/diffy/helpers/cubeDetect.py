import numpy as np
import cv2


cap = cv2.VideoCapture(0)

if (cap.isOpened() == False):
    print("Error opening video stream")

while (cap.isOpened()):
    ret, frame = cap.read()

    if ret == True:
        # convert image to HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # define HSV range of purple
        lower_purple = np.array([110, 100, 100])
        upper_purple = np.array([150, 255, 255])

        # threshold the HSV image for purple
        threshed_img = cv2.inRange(hsv_img, lower_purple, upper_purple)
        converted = cv2.cvtColor(threshed_img, cv2.COLOR_BGR2RGB)


        # smooth image with erosion, dilation, and smooth gaussian
        # 5x5 pixel kernel

        kernel = np.ones((11, 11), np.uint8)

        # erode and dilate
        threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations=3)
        threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=2)

        # more smoothing to account for reflection and other noises
        smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations=11)
        smoothed_img = cv2.erode(smoothed_img, kernel, iterations=9)

        # detect all edges within the image
        edges_img = cv2.Canny(smoothed_img, 100, 200)
        contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # set parameters for text and bounding box
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        fontColor = (0, 0, 255)
        lineType = 2

        # determine if cube
        for cnt in contours:
            boundingRect = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.06 * cv2.arcLength(cnt, True), True)
            # if contour is rectangle, draw bounding box

            print(len(approx))
            if 3 < len(approx) < 6:
                x, y, w, h = cv2.boundingRect(approx)
                rect = (x, y, w, h)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                bottomLeftCornerOfText = (x, y)
                cv2.putText(frame, 'cube',
                            bottomLeftCornerOfText,
                            font,
                            fontScale,
                            fontColor,
                            lineType)

        # display the result frame
        cv2.imshow('frame', frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break


cap.release()
cv2.destroyAllWindows()