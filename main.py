import os
import cv2
from cvzone.HandTrackingModule import HandDetector
import numpy as np


width , height = 2420,1080
folderPath = "Data"

cap = cv2.VideoCapture(0)
cap.set(3,width)
cap.set(4,height)

pathImages = sorted(os.listdir(folderPath),key=len)

print(pathImages)

imgNumber = 0
hs, ws = int(220*1),int(413*1)

detector = HandDetector(detectionCon = 0.8,maxHands=1)
gestureThreshold = 400
buttonPress = False
buttonCounter = 0
buttonDelay = 10
annotations = [[]]
annotationNumber = -1
annotationStart = False

while True:
    success,img = cap.read()
    img = cv2.flip(img,1)
    pathFullImage = os.path.join(folderPath,pathImages[imgNumber]) 
    imgCurrent = cv2.imread(pathFullImage)

    hands,img = detector.findHands(img,flipType=False)
    cv2.line(img,(0,gestureThreshold),(width,gestureThreshold),(30,144,255),1)

    if hands and buttonPress is False:
        hand = hands[0]
        fingers = detector.fingersUp(hand)
        cx,cy = hand['center']
        lmList = hand['lmList']
        indexFinger = lmList[8][0],lmList[8][1]
        xVal = int(np.interp(lmList[8][0],[width // 4, w],[0,width]))
        yVal = int(np.interp(lmList[8][1],[150,height-150],[0,height]))
        indexFinger = xVal,yVal
        print(fingers)

        if cy <= gestureThreshold:
            annotationStart = False
            if fingers == [0,0,0,0,0]:
                print("Left")
                
                if imgNumber >0:
                    buttonPress = True
                    annotations = [[]]
                    annotationNumber = -1
                    annotationStart = False
                    imgNumber -=1
            if fingers == [0,1,0,0,0]:
                print("Right")
                
                if imgNumber < len(pathImages)-1:
                    buttonPress = True
                    annotations = [[]]
                    annotationNumber = -1
                    annotationStart = False
                    imgNumber += 1
        #show pointer
        if fingers == [1,1,1,0,0]:
            cv2.circle(imgCurrent,indexFinger,12,(0,0,255),cv2.FILLED)
            annotationStart = False

        #draw pointer
        if fingers == [1,1,0,0,0]:
            if annotationStart is False:
                annotationStart = True
                annotationNumber += 1
                annotations.append([])
            cv2.circle(imgCurrent, indexFinger,12, (0,0,255),cv2.FILLED)
            annotations[annotationNumber].append(indexFinger)
        else:
            annotationStart = False

        if fingers == [0,1,1,1,1]:
            if annotations:
                annotations.pop(-1)
                annotationNumber -=1
                buttonPress = True
    
    else:
        annotationStart = False


    if buttonPress:
        buttonCounter += 1
        if buttonCounter > buttonDelay:
            buttonCounter = 0
            buttonPress = False

    for i in range (len(annotations)):
        for j in range (len(annotations[i])):
            if j!=0:
                cv2.line(imgCurrent,annotations[i][j-1],annotations[i][j],(0,0,200),12)

    imgSmall = cv2.resize(img,(ws,hs))
    try:
        h, w, _ = imgCurrent.shape
    except AttributeError:
        print("AttributeError: Value is",img)
    imgCurrent[0:hs,w-ws:w] = imgSmall
    
    cv2.imshow('Image',img)
    cv2.imshow("Slides",imgCurrent)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break