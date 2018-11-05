import cv2
import numpy as np
refPt = []
cropping = False        
def click_and_crop(event, x, y, flags, param):

    global refPt, cropping
    if event == cv2.EVENT_LBUTTONDOWN:
      if cropping is True:
        print "HERE"
        refPt.append((x, y))
        cv2.rectangle(clone1, refPt[0], refPt[1], (0, 255, 0), 2)
      refPt = [(x, y)]
      cropping = True
    elif event == cv2.EVENT_LBUTTONUP:
        refPt.append((x, y))
        cropping = False
        cv2.rectangle(clone1, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", clone1)

image = cv2.imread('orig.jpg')
x,y,z=np.shape(image)
clone1=cv2.resize(image,(y/2,x/2),interpolation=cv2.INTER_AREA)

clone=clone1.copy()
cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)
while True:
    cv2.imshow("image", clone1)
    key=cv2.waitKey(1) & 0xFF
    if key == ord("r"):
        clone1 = clone.copy()
    if key == ord("c"):
        break
if len(refPt) == 2:
  a,b,c,d=(refPt[0][1])*2,(refPt[1][1])*2,(refPt[0][0])*2,(refPt[1][0])*2
  roi = image[a:b, c:d]
  cv2.imshow("ROI", roi)
  cv2.waitKey(0)
  print('Cropping Complete')
  cv2.destroyAllWindows()
