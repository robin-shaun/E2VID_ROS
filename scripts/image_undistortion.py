import cv2
import os
import numpy as np

K = np.array([[540.8107248682578, 0, 178.37790385239077], [0, 540.2890152484031, 135.20088424862703], [0, 0, 1]])
D = np.array([-0.15691417868518728, -0.013178018060342524, 7.144080498358582e-06,
    0.006769107448204392])
ImgWidth = 346
ImgHeight = 260
imageSize = (ImgWidth, ImgHeight)
alpha = 0
NewCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, 0)
print(NewCameraMatrix)
map1, map2 = cv2.initUndistortRectifyMap(K, D, None, NewCameraMatrix, imageSize, cv2.CV_16SC2)
directory = "./output/spinning_1/reconstruction/"
if not os.path.isdir(directory+"images_undistortion/"):
    os.mkdir(directory+"images_undistortion/")
for filename in os.listdir(directory+"images/"):
    RawImage = cv2.imread(directory+"images/"+filename,cv2.IMREAD_GRAYSCALE)
    if RawImage is None:
        continue
    UndistortImage = cv2.remap(RawImage, map1, map2, cv2.INTER_LINEAR)
    UndistortImageName = directory+"images_undistortion/"+ filename.split('.')[0] + '_undistortion.png'
    cv2.imwrite(UndistortImageName, UndistortImage)