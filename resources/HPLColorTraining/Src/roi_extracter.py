#!/usr/bin/python

import sys, getopt
import cv2 
import glob
import numpy as np
  
refPt = []
cropping = False

f = open("dataset.txt", "a+") 
f.close()

def click_and_crop(event, x, y, flags, param):
    global refPt, cropping
    if event == cv2.EVENT_MOUSEMOVE:
      if cropping is True:
        global im
        im = clone.copy()
        if abs(x - refPt[0][0]) >= 10 and abs(y - refPt[0][1]) >= 10:
          cv2.rectangle(im, refPt[0], (x, y), (0, 255, 0), 2)
    elif event == cv2.EVENT_LBUTTONDOWN:
        refPt = [(x, y)]
        cropping = True
    elif event == cv2.EVENT_LBUTTONUP:
        refPt.append((x, y))
        cropping = False
        cv2.rectangle(im, refPt[0], refPt[1], (0, 255, 0), 2)
        cv2.imshow("image", im)

def extract_rois(filePath):
  rects = []
  fromCenter = False
  files = filePath + '/*.jpg'
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", click_and_crop)
  for filename in glob.glob(files):
    # Read image file
    global im
    im=cv2.imread(filename)
    imcount = 0
    while True:
      goToNext = False
      #if imcount > 0:
      while True:
        cv2.imshow("image", im)
        cv2.waitKey(1) & 0xFF
        cmd = raw_input("1. Press 'r' to continue with previous image...\n2. Press Enter to go to next image...")
        if cmd == "r":
          break
        elif cmd == "":
          goToNext = True
          break
        else:
          print "Wrong input"
      
      if goToNext:
        break
      
      global clone
      clone=im.copy()
      while True:
        cv2.imshow("image", im)
        key=cv2.waitKey(1) & 0xFF
        if key == ord("c"):
          im = clone.copy()
          break
      if len(refPt) == 2:
        a,b,c,d=(refPt[0][1]),(refPt[1][1]),(refPt[0][0]),(refPt[1][0])
        if a > b:
          tmp = b
          b = a
          a = tmp
        if c > d:
          tmp = d
          d = c
          c = tmp
        imCrop = im[a:b, c:d]
        cv2.imshow("Crop", imCrop)
        cv2.waitKey(0)
        print('Cropping Complete')
      else:
        continue
      
      f = open("dataset.txt", "a")
      while True:
        labelName = ''
        label = raw_input("Label for this roi:\n1. Green\n2. White\n3. Black\n4. Red\n5. Yellow\n6. Blue\n7. Gray\n8. Orange\n")
        if label == '1':
          labelName = 'Green'
          imcount += 1
          break
        elif label == '2':
          labelName = 'White'
          imcount += 1
          break
        elif label == '3':
          labelName = 'Black'
          imcount += 1
          break
        elif label == '4':
          labelName = 'Red'
          imcount += 1
          break
        elif label == '5':
          labelName = 'Yellow'
          imcount += 1
          break
        elif label == '6':
          labelName = 'Blue'
          imcount += 1
          break
        elif label == '7':
          labelName = 'Gray'
          imcount += 1
          break
        elif label == '8':
          labelName = 'Orange'
          imcount += 1
          break
        else:
          print "Wrong label entered. Enter 1-8 to assign that label."
          continue
        print filename
      f.write(filename + " " + str(a) + " " + str(b) + " " + str(c) + " " + str(d) + " " + labelName + "\n") 
      f.close()
      
def print_usage():
  print 'Usage: python roi_extracter.py -d <pathToDir>'

def main(argv):
  filePath = ''
  try:
    opts, args = getopt.getopt(argv,"hd:",["dir"])
    if len(opts) < 1:
      print "Wrong number of arguments..."
      print_usage()
      sys.exit()
  except getopt.GetoptError:
    print_usage()
    sys.exit(2)
  for opt, arg in opts:
    if opt == '-h':
       print_usage()
       sys.exit()
    elif opt in ("-d", "--dir"):
       filePath = arg
    else:
      print_usage()
      sys.exit()
  print 'Reading dataset images from the directory:\n ', filePath
  extract_rois(filePath)

if __name__ == "__main__":
   main(sys.argv[1:])
