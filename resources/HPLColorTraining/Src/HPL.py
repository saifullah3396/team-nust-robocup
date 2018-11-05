#!/usr/bin/python

import sys, getopt
import cv2 
import glob
import numpy as np
import math 

#green
#rMin = 5
#rMax = 10
#densityThres = 1.0
#alpha = 0.8

rMin = 5
rMax = 20
densityThres = 0.05
alpha = 0.8

class Prototype:
  classCounter = 0
  def __init__(self, center, radius):
    Prototype.classCounter += 1
    self.identity = Prototype.classCounter
    self.center = center
    self.radius = radius
    self.N = 1
    self.discard = False

class PixelSample:
  def __init__(self):
      self.proto = 0
      self.label = 0
      self.value = np.zeros((3,1))
      
def learn(dataset, labelString):
  #table = np.zeros((255,255,255))
  samples = []
  with open(dataset) as file:
    for l in file:
      line = l.split()
      if labelString != line[5]:
        continue
      im=cv2.imread(line[0])
      yuv = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
      for i in range(int(line[1]), int(line[2])):
        for j in range(int(line[3]), int(line[4])):
          p = PixelSample()
          p.value[0] = yuv[i,j,0]
          p.value[1] = yuv[i,j,1]
          p.value[2] = yuv[i,j,2]
          samples.append(p)
  
  #for sample in samples:
  #  print sample.value[0], sample.value[1], sample.value[2]
  
  # HPL Learning
  prototypes = []
  r = rMax
  while r > rMin: 
    for sample in samples:
      if sample.label == 0:
        for prototype in prototypes:
          if prototype.discard:
            continue
          d = np.linalg.norm(sample.value-prototype.center)
          if d <= prototype.radius:
            prototype.N += 1
            sample.proto = prototype.identity
            break
            
        if sample.proto == 0:
          prototypes.append(Prototype(sample.value, r))
    
    for prototype in prototypes:
      if prototype.discard:
        continue
      density = 3 * prototype.N / (4 * math.pi * prototype.radius**3)
      print density, r
      if density > densityThres:
        for sample in samples:
          if sample.proto == prototype.identity:
            sample.label = prototype.identity
      else:
        while prototype.radius > rMin:
          prototype.radius = alpha * prototype.radius
          if density > densityThres:
            for sample in samples:
              if sample.proto == prototype.identity:
                sample.label = prototype.identity
            break
        if prototype.radius < rMin:
          prototype.discard = True
          for sample in samples:
            if sample.proto == prototype.identity:
              sample.prototype = 0
              sample.label = 0
    r = alpha * r
  
  for prototype in prototypes:
    if prototype.discard:
      continue
    print "center" + str(prototype.center)
    print "radius" + str(prototype.radius)

  #for i in range(255):
  #  for j in range(255):
  #    for k in range(255):
  #      for prototype in prototypes:
  #          if prototype.discard:
  #            continue
  #          d = np.linalg.norm((i, j, k)-prototype.center)
  #          if d <= prototype.radius:
  #            table[i,j,k] = 1
  #print "DONE"
  lastIm = ''
  with open(dataset) as file:
    for l in file:
      line = l.split()
      if line[0] == lastIm:
        continue
      im=cv2.imread(line[0])
      yuv = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
      rows,cols,nslice = im.shape
      for i in range(rows):
        for j in range(cols):
          p = PixelSample()
          p.value[0] = yuv.item(i,j,0)
          p.value[1] = yuv.item(i,j,1)
          p.value[2] = yuv.item(i,j,2)
          for prototype in prototypes:
            if prototype.discard:
              continue
            d = np.linalg.norm(p.value-prototype.center)
            if d <= prototype.radius:
              im[i,j,0] = 0
              im[i,j,1] = 255
              im[i,j,2] = 0
      cv2.imshow("image", im)
      cv2.waitKey(0)
      lastIm = line[0]
            
  print "End of loop"

def print_usage():
  print 'Usage: python HPL.py -d <dataset.txt> -l <label>'

def main(argv):
  dataset = ''
  try:
    opts, args = getopt.getopt(argv,"hd:l:",["dataset", "label"])
    if len(opts) < 2:
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
    elif opt in ("-d", "--dataset"):
       dataset = arg
    elif opt in ("-l", "--label"):
       labelString = arg
    else:
      print_usage()
      sys.exit()
  print 'Reading dataset from the file:\n ', dataset
  learn(dataset, labelString)

if __name__ == "__main__":
   main(sys.argv[1:])
