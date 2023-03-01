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

        # define yellow HSV range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])

        # threshold the image to get only yellow pixels
        threshed_img = cv2.inRange(hsv_img, lower_yellow, upper_yellow)
        converted = cv2.cvtColor(threshed_img, cv2.COLOR_BGR2RGB)


        # smooth image with erosion, dilation, and smooth gaussian
        kernel = np.ones((5, 5), np.uint8)

        # eroding and dilate
        threshed_img_smooth = cv2.erode(threshed_img, kernel, iterations=3)
        threshed_img_smooth = cv2.dilate(threshed_img_smooth, kernel, iterations=2)

        # further smoothing
        smoothed_img = cv2.dilate(threshed_img_smooth, kernel, iterations=11)
        smoothed_img = cv2.erode(smoothed_img, kernel, iterations=7)

        # detect all edges with canny edge detection
        edges_img = cv2.Canny(smoothed_img, 100, 200)
        contours, hierarchy = cv2.findContours(edges_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # set parameters for writing text and drawing lines
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 2
        fontColor = (0, 0, 255)
        lineType = 2

        for cnt in contours:
            boundingRect = cv2.boundingRect(cnt)
            approx = cv2.approxPolyDP(cnt, 0.06 * cv2.arcLength(cnt, True), True)
            # if contour is a triangle, draw a bounding box and label cone
            print(len(approx))

            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                rect = (x, y, w, h)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
                bottomLeftCornerOfText = (x, y)
                cv2.putText(frame, 'cone',
                            bottomLeftCornerOfText,
                            font,
                            fontScale,
                            fontColor,
                            lineType)

        # display processed frame
        cv2.imshow('frame', frame)

        if cv2.waitKey(25) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()