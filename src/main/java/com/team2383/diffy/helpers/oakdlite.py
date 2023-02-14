import depthai as dai
import cv2
import numpy as np 

def detectCircles(img, threshold, region, radius=None):
    (M,N) = img.shape
    if radius == None:
        R_max = np.max(M,N)
        R_min = 3
    else:
        [R_max, R_min] = radius
    
    R = R_max - R_min

    #Accumulator array with r, x, and y coordinates
    A = np.zeroes((R_max, M+2*R_max, N+2*R_max))
    B = np.zeroes((R_max, M+2*R_max, N+2*R_max))

    theta = np.arange(0,360)*np.pi/180
    edges = np.argwhere(img[:,:])

    for val in range(R):
        r=R_min+val

        bprint=np.zeroes((2*(r+1),2*(r+1)))
        (m,n)= (r+1,r+1)

        for angle in theta:
            x = int(np.round(r*np.cos(angle)))
            y = int(np.round(r*np.sin(angle)))
            bprint[m+x,n+y] = 1
        constant = np.argwhere(bprint).shape[0]

        for x,y in edges:
            X = [x-m+R_max,x+m+R_max]                                
            Y= [y-n+R_max,y+n+R_max]                       
            A[r,X[0]:X[1],Y[0]:Y[1]] += bprint
        A[r][A[r]<threshold*constant/r] = 0

    for r,x,y in np.argwhere(A):
        temp = A[r-region:r+region,x-region:x+region,y-region:y+region]
        try:
            p,a,b = np.unravel_index(np.argmax(temp),temp.shape)
        except:
            continue
        B[r+(p-region),x+(a-region),y+(b-region)] = 1

    return B[:,R_max:-R_max,R_max:-R_max]

        


fullFrameTracking = args.full_frame
ppline = dai.Pipeline()

#start pipeline and shiiii
monoLeft = ppline.create(dai.node.MonoCamera)
rgbCam = ppline.create(dai.node.ColorCamera)
monoRight = ppline.create(dai.node.MonoCamera)

edgeDetectorLeft = ppline.create(dai.node.EdgeDetector)
edgeDetectorRGB = ppline.create(dai.node.EdgeDetector)
edgeDetectorRight = ppline.create(dai.node.EdgeDetector)

xoutEdgeLeft = ppline.create(dai.node.XLinkOut)
xoutEdgeRGB = ppline.create(dai.node.XLinkOut)
xoutEdgeRight = ppline.create(dai.node.XLinkOut)
xinEdgeCfg = ppline.create(dai.node.XLinkIn)

xoutEdgeLeft.SetStreamName("edge left")
xoutEdgeRGB.SetStreamName("edge rgb")
xoutEdgeRight.SetStreamName("edge right")
xinEdgeCfg.SetStreamName("edge cfg")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

rgbCam.setBoardSocket(dai.CameraBoardSocket.RGB)
rgbCam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

edgeDetectorRGB.setMaxOutputFrameSize(rgbCam.getVideoWidth() * rgbCam.getVideoHeight())

#linking buddy
monoLeft.out.link(edgeDetectorLeft.inputImage)
rgbCam.video.link(edgeDetectorRGB.inputImage)
monoRight.out.link(edgeDetectorRight.inputImage)

edgeDetectorLeft.outputImage.link(xoutEdgeLeft.input)
edgeDetectorRGB.outputImage.link(xoutEdgeRGB.input)
edgeDetectorRight.outputImage.link(xoutEdgeRight.input)

xinEdgeCfg.out.link(edgeDetectorLeft.inputConfig)
xinEdgeCfg.out.link(edgeDetectorRGB.inputConfig)
xinEdgeCfg.out.link(edgeDetectorRight.inputConfig)

with dai.Device(ppline) as device:
    edgeLeftQueue = device.getOutputQueue("edge left", 8, False)
    edgeRGBQueue = device.getOutputQueue("edge rgb", 8, False)
    edgeRightQueue = device.getOutputQueue("edge right", 8, False)
    edgeCfgQueue = device.getInputQueue("edge cfg")

    while(True):
        edgeLeft = edgeLeftQueue.get()
        rgbEdge = edgeRGBQueue.get()
        edgeRight = edgeRightQueue.get()

        edgeLeftFrame = edgeLeft.getFrame()
        rgbEdgeFrame = rgbEdge.getFrame()
        edgeRightFrame = edgeRight.getFrame

        #gaussian blur
        gaussianLeft = cv2.GaussianBlur(edgeLeftFrame, (7,7), 0)
        gaussianRGB = cv2.GaussianBlur(rgbEdgeFrame, (7,7), 0)
        gaussianRight = cv2.GaussianBlur(edgeRightFrame, (7,7), 0)

        #yellow filtering
        hsv = cv2.cvtColor(gaussianRGB, cv2.COLOR_BGR2HSV)
        lowerYellow = np.array([1,2,3])
        upperYellow = np.array([4,5,6])

        mask = cv2.inRange(hsv, lowerYellow, upperYellow)
        rgbFilter = cv2.bitwise_and(gaussianRGB, gaussianRGB, mask=mask)

        #detecting cone and doing sobel filter
        coneDetect = detectCircles(rgbFilter, 8.1, 15) #edge image, thresholding val, region size detecting peaks
        cfg = dai.EdgeDetectorConfig()
        sobelHorizontalKernel = [[1, 0, -1], [2, 0, -2], [1, 0, -1]]
        sobelVerticalKernel = [[1, 2, 1], [0, 0, 0], [-1, -2, -1]]
        cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel)
        edgeCfgQueue.send(cfg)
        
        #showing image
        cv2.imshow("edge rgb", coneDetect)

        key = cv2.waitKey(1)
        if key == 'q':
            break


        



