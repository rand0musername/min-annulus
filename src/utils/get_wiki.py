import cv2
import numpy as np

# extracts a testcase from wiki.png

img = cv2.imread('wiki.png', cv2.IMREAD_COLOR)
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_,bw = cv2.threshold(gray,1,255,cv2.THRESH_BINARY)

kernel = np.ones((6, 6), np.uint8)
erosion = cv2.dilate(bw, kernel, iterations = 1)

tup = np.where(erosion < 255)
l = list(zip(tup[0], tup[1]))
print(len(l))
for pt in l:
    print('{} {}'.format(pt[1], -pt[0]))
