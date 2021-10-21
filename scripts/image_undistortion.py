import cv2
import os
import numpy as np

K = np.array([[514.184486265331, 0.0, 173.29979747202148],[0.0, 514.6561978303288, 102.15522967918119],[0, 0, 1]])
D = np.array([-0.17556400034969583, 0.045659767313701416, 0.00899513002963119, -0.0028183431453483662, 0.0])
ImgWidth = 346
ImgHeight = 260
imageSize = (ImgWidth, ImgHeight)
alpha = 0
NewCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0)
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, NewCameraMatrix, imageSize, cv2.CV_16SC2)
directory = "./output/indoor9/reconstruction/"
os.mkdir(directory+"images_undistortion/")
for filename in os.listdir(directory+"images/"):
    RawImage = cv2.imread(directory+"images/"+filename,cv2.IMREAD_GRAYSCALE)
    if RawImage is None:
        continue
    UndistortImage = cv2.remap(RawImage, map1, map2, cv2.INTER_LINEAR)
    UndistortImageName = directory+"images_undistortion/"+ filename.split('.')[0] + '_undistortion.png'
    cv2.imwrite(UndistortImageName, UndistortImage)