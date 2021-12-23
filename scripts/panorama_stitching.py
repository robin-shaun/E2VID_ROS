# import the necessary packages
from Stitcher import Stitcher
import argparse
import imutils
import cv2
# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-f", "--first", required=True,
	help="path to the first image")
ap.add_argument("-s", "--second", required=True,
	help="path to the second image")
args = vars(ap.parse_args())
# load the two images and resize them to have a width of 400 pixels
# (for faster processing)
imageA = cv2.imread(args["first"])
imageB = cv2.imread(args["second"])
imageA = imutils.resize(imageA, width=400)
imageB = imutils.resize(imageB, width=400)
# stitch the images together to create a panorama

#Calculate H
R = [9.99990798e-01, -6.32492385e-04, -4.24307214e-03, -7.30597639e-02,
     6.44736387e-04,  9.99995631e-01,  2.88489843e-03, -1.23275257e-03,
     4.24122892e-03, -2.88760755e-03,  9.99986837e-01, -1.10420407e-03]
stitcher = Stitcher()
(result, vis) = stitcher.stitch([imageA, imageB], None, showMatches=True)
# show the images
cv2.imshow("Image A", imageA)
cv2.imshow("Image B", imageB)
cv2.imshow("Keypoint Matches", vis)
cv2.imshow("Result", result)
cv2.waitKey(0)