# -*- coding: utf-8 -*-
import cv2
import numpy as np
import matplotlib.pyplot as plt


def watershed(image_path):

    image_name = image_path.split("/")[-1]
    image = cv2.imread(image_path)
    # image = multiScaleSharpen(image, 5)
    # 前提：降噪
    # blurred = cv2.pyrMeanShiftFiltering(image, 25, 100)
    # 第一步：灰度处理
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', gray)
    # cv2.waitKey(0)

    
    img_medianBlur=cv2.medianBlur(gray,5)
    cv2.imshow('img_medianBlur', img_medianBlur)
    cv2.waitKey(0)

    

    # 第二步：二值化处理 + 反色处理 255 -> 0 | 0 -> 255
    # ret, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    ret, binary = cv2.threshold(img_medianBlur, 1, 255, cv2.THRESH_BINARY)


    # 对二值化的结果执行开运算

    # noise removal
    kernel = np.ones((3, 3), np.uint8)
    opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=2)
    # cv2.imshow('opening', opening)
    # cv2.waitKey(0)

    # sure background area
    sure_bg = cv2.dilate(opening, kernel, iterations=3)

    # Finding sure foreground area
    dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
    # 通过距离变换的结果取二值化，得到前景
    ret, sure_fg = cv2.threshold(dist_transform, 0.5 * dist_transform.max(),
                                 255, 0)

    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv2.subtract(sure_bg, sure_fg)

    # Marker labelling
    ret, markers = cv2.connectedComponents(sure_fg)

    # Add one to all labels so that sure background is not 0, but 1
    markers = markers + 1

    # Now, mark the region of unknown with zero
    markers[unknown == 255] = 0
    sure_bg[unknown == 255] = 0
    sure_bg[sure_bg == 255] = 2
    sure_bg = sure_bg.astype(np.int32)

    # plt.figure(1)
    # plt.title("sure_bg")
    # plt.imshow(sure_bg)

    # 分水岭只是对0的位置进行分割 1-背景 0-待分割 2-前景
    result = cv2.watershed(image, markers=markers)

    # 分水岭结果标记轮廓为红色
    image[result == -1] = [255, 0, 0]
    # cv2.imwrite("./watershed_res.jpg", result)

    plt.figure(0)

    plt.subplot(231)
    plt.title("binary")
    plt.imshow(binary)

    plt.subplot(232)
    plt.title("seed new")
    plt.imshow(opening)

    plt.subplot(233)
    plt.title("distance")
    plt.imshow(dist_transform * 50)

    plt.subplot(234)
    plt.title("seed ori")
    plt.imshow(markers)

    plt.subplot(235)
    plt.title("result markers")
    plt.imshow(result)

    plt.subplot(236)
    plt.title("watershed")
    plt.imshow(image)

    # plt.savefig(os.path.join(output_dir, "watershed_" + image_name))
    plt.show()


watershed("/home/robin/event_based_vision/rpg_e2vid/output/circle/reconstruction/events/events_0007925476.png")
