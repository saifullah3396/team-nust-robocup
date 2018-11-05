/**
 * @file VisionModule/include/FeatureExtraction/RegionSegmentation.cpp
 *
 * This file implements the class for segmenting the image into different
 * regions for further processing.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author AbdulRehman
 * @date 5 Mar 2018
 */

#include "VisionModule/include/FeatureExtraction/FeatureExtraction.h"
#include "VisionModule/include/FeatureExtraction/FieldExtraction.h"
#include "VisionModule/include/FeatureExtraction/BallExtraction.h"
#include "VisionModule/include/FeatureExtraction/GoalExtraction.h"
#include "VisionModule/include/FeatureExtraction/RegionSegmentation.h"

RegionSegmentation::RegionSegmentation(VisionModule* visionModule) :
  FeatureExtraction(visionModule), DebugBase("RegionSegmentation", this)
{
  initDebugBase();
  int tempSendTime;
  int tempDrawRegions;
  GET_CONFIG(
    "VisionDebug",
    (int, RegionSegmentation.sendTime, tempSendTime), (int, RegionSegmentation.drawRegions, tempDrawRegions), (int, RegionSegmentation.scanStepLow, scanStepLow), (int, RegionSegmentation.scanStepHigh, scanStepHigh), )
  SET_DVAR(int, sendTime, tempSendTime);
  SET_DVAR(int, drawRegions, tempDrawRegions);
  vScanLimitIdx = getImageHeight() - scanStepLow;
  hScanLimitIdx = getImageWidth() - scanStepLow;
}

void
RegionSegmentation::processImage()
{
  /*static vector<int> l(3), u(3);
   VisionUtils::createWindow("Color-Calibration");
   VisionUtils::addTrackBar("Color-Lowery", "Color-Calibration", &l[0], 255);
   VisionUtils::addTrackBar("Color-Loweru", "Color-Calibration", &l[1], 255);
   VisionUtils::addTrackBar("Color-Lowerv", "Color-Calibration", &l[2], 255);
   VisionUtils::addTrackBar("Color-Uppery", "Color-Calibration", &u[0], 255);
   VisionUtils::addTrackBar("Color-Upperu", "Color-Calibration", &u[1], 255);
   VisionUtils::addTrackBar("Color-Upperv", "Color-Calibration", &u[2], 255);	
   Mat binary;
   VisionUtils::applyThreshold(
   yuv, 
   l,
   u,
   binary
   );
   VisionUtils::displayImage(binary, "binary1");
   */
  //Mat yuv = makeYuvMat();
  //Mat green, white;
  /*green = Mat(yuv.size(), CV_8UC3, Scalar(0,0,0));
   for(int y = 0; y < getImageHeight(); ++y)
   {
   for (int x = 0; x < getImageWidth(); ++x) {
   auto color = getYUV(x, y);
   if(colorHandler->isGreenHist(color)) {
   green.at<Vec3b>(y, x) = Vec3b(0, 255, 0);
   }
   }
   }*/
  //colorHandler->getBinary(yuv, green, Colors::GREEN);
  /*VisionUtils::displayImage(yuv, "yuv");
   waitKey(0);
   VisionUtils::displayImage(green, "greens");
   waitKey(0);
   colorHandler->getBinary(yuv, white, Colors::WHITE);
   Mat g2;
   colorHandler->getBinary(yuv, g2, Colors::GREEN);*/
  //colorHandler->getBinary(yuv, white, Colors::WHITE);
  //colorHandler->getBinary(yuv, green, Colors::GREEN);
  //VisionUtils::displayImage(yuv, "yuvyuv");
  //VisionUtils::displayImage(green, "green");
  //VisionUtils::displayImage(white, "w");
  //waitKey(0);
#ifdef DEBUG_BUILD
  auto tStart = high_resolution_clock::now();
#endif

  //whiteEdges = Scalar(0);
  //lineEdges.clear();

  horRobotLines.clear();
  //horBallLines.clear();
  horJerseyLinesOurs.clear();
  horJerseyLinesOpps.clear();

  //verGoalLines.clear();
  verRobotLines.clear();
  //verBallLines.clear();
  verJerseyLinesOurs.clear();
  verJerseyLinesOpps.clear();

  borderPoints.clear();
  int horPrevLen = 1000;
  ourColor = Colors::YELLOW;
  oppColor = Colors::BLUE;

  //! Last field height with some margin
  auto lastHeight = fieldExt->getFieldHeight() - 50;
  
  srand(time(0));
  int horStartHigh = rand() % scanStepHigh;

  //! Field scan parameters
  int otherF = 0, greenF = 0, horStartF = -1, horEndF = -1;
  int fieldMin = getImageWidth();
  int fieldMax = 0;
  bool fieldFound = false;
  //! Robot scan parameters
  int whiteR = 0, otherR = 0, greenR = 0, horStartRobot = -1;
  //! Ball scan parameters
  int otherB = 0, blackB = 0, whiteB = 0, horStartBall = -1;
  //! Team jersey scan parameters
  int horStartJerseyOurs = -1;
  //! Opponent jersey scan parameters
  int horStartJerseyOpps = -1;
  for (int y = horStartHigh; y < getImageHeight(); y = y + scanStepHigh) {
    bool scanInField = y > lastHeight;
    if (scanInField) {
      otherF = 0, greenF = 0, horStartF = -1, horEndF = -1;
      fieldMin = getImageWidth();
      fieldMax = 0;
      fieldFound = false;
      whiteR = 0, otherR = 0, greenR = 0, horStartRobot = -1;
      //! Ball scan parameters
      otherB = 0, blackB = 0, whiteB = 0, horStartBall = -1;
    }
    //! Team jersey scan parameters
    int horStartJerseyOurs = -1;
    //! Opponent jersey scan parameters
    int horStartJerseyOpps = -1;
    //uchar* p = whiteEdges.ptr<uchar>(y);
    //auto tStart1 = high_resolution_clock::now();
    int horStartLow = rand() % scanStepLow;
    for (int x = horStartLow; x < getImageWidth(); x = x + scanStepLow) {
      auto color = getYUV(x, y);
      //! Horizontal Scanning
      if (colorHandler->isColor(color, Colors::GREEN)) {
        if (scanInField) {
          //! Field Scanning
          if (horStartF == -1) {
            horStartF = x;
          }
          otherF = 0;
          greenF++;

          greenR++;
          if (greenR > 1) {
            if (otherR != 0) {
              float rR = whiteR / (float) otherR;
              if (horStartRobot != -1 && otherR > 0 && whiteR > 0 && rR > 0.05f && rR < 0.99f) {
                auto sl = boost::make_shared < ScannedLine > (Point(
                  horStartRobot,
                  y), Point(x - scanStepLow, y));
                horRobotLines.push_back(sl);
                // line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(0,0,255), 1);
                //  imshow("image", bgrMat[currentImage]);
                //  waitKey(0);
              }
              otherR = 0;
              greenR = 0;
              whiteR = 0;
              horStartRobot = -1;
            }
            horStartRobot = -1;
          }

        }

        //! Team Jersey Scanning
        if (horStartJerseyOurs != -1) {
          auto sl = boost::make_shared < ScannedLine > (Point(
            horStartJerseyOurs,
            y), Point(x - scanStepLow, y));
          horJerseyLinesOurs.push_back(sl);
          //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,255,0), 2);
          horStartJerseyOurs = -1;
        }

        //! Opponent Jersey Scanning
        if (horStartJerseyOpps != -1) {
          auto sl = boost::make_shared < ScannedLine > (Point(
            horStartJerseyOpps,
            y), Point(x - scanStepLow, y));
          horJerseyLinesOpps.push_back(sl);
          //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,0,0), 2);
          horStartJerseyOpps = -1;
        }
      } else {
        bool isOurs = colorHandler->isColor(color, ourColor);
        bool isOpps = colorHandler->isColor(color, oppColor);

        if (scanInField) {
          bool isWhite = colorHandler->isColor(color, Colors::WHITE);
          bool isBlack = colorHandler->isColor(color, Colors::BLACK);
          //! Field Scanning
          if (horStartF != -1) otherF++;

          /*if (scanBall) {
           //! Ball Scanning
           if (horStartBall == -1) {
           horStartBall = x;
           } else {
           otherB++;
           if (isWhite) {
           whiteB++;
           }
           if (isBlack) {
           blackB++;
           }
           }
           }*/

          //! Robot Scanning
          if (horStartRobot == -1) {
            horStartRobot = x;
          } else {
            otherR++;
            if (isWhite) {
              whiteR++;
            }
            /*if (whiteR > 50 && otherR < 1) {
             horStartRobot = x;
             whiteR = 0;
             otherR = 0;
             greenR = 0;
             }*/
          }
          greenR = 0;
        }

        //! Team Jersey Scanning
        if (horStartJerseyOurs == -1) {
          if (isOurs) horStartJerseyOurs = x;
        }

        //! Team Jersey Scanning
        if (!isOurs) {
          if (horStartJerseyOurs != -1) {
            auto sl = boost::make_shared < ScannedLine > (Point(
              horStartJerseyOurs,
              y), Point(x - scanStepLow, y));
            horJerseyLinesOurs.push_back(sl);
            //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,255,0), 2);
            horStartJerseyOurs = -1;
          }
        }

        //! Opponent Jersey Scanning
        if (horStartJerseyOpps == -1) {
          if (isOpps) horStartJerseyOpps = x;
        }

        //! Opponent Jersey Scanning
        if (!isOpps) {
          if (horStartJerseyOpps != -1) {
            auto sl = boost::make_shared < ScannedLine > (Point(
              horStartJerseyOpps,
              y), Point(x - scanStepLow, y));
            horJerseyLinesOpps.push_back(sl);
            //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,0,0), 2);
            horStartJerseyOpps = -1;
          }
        }
      }

      if (scanInField) {
        //! Field Scanning
        if (otherF > 2 || x >= hScanLimitIdx) {
          if (horStartF != -1) {
            if (greenF > 2) {
              if (abs(horStartF - horEndF) > horPrevLen) {
                horEndF = x;
                fieldMin = horStartF;
                fieldMax = horEndF;
                fieldFound = true;
                horPrevLen = horEndF - horStartF;
              } else {
                horEndF = x;
                fieldMin = horStartF < fieldMin ? horStartF : fieldMin;
                fieldMax = horEndF > fieldMax ? horEndF : fieldMax;
                fieldFound = true;
                horPrevLen = horEndF - horStartF;
              }
              horStartF = -1;
              greenF = 0;
            } else {
              horStartF = -1;
              greenF = 0;
            }
          }
          horStartF = -1;
          greenF = 0;
        }
      }
    }
    //! Field points accumulation
    if (fieldFound) {
      borderPoints.push_back(Point(fieldMin, y));
      borderPoints.push_back(Point(fieldMax, y));
      //line(bgrMat[currentImage], Point(fieldMin, y), Point(fieldMax, y), Scalar(0,255,0), 1);
    }
    //auto tEnd1 = high_resolution_clock::now();
    //duration<double> timeSpan1 = tEnd1 - tStart1;
    //cout << "innertime = " << timeSpan1.count() << endl;
  }

  avgHeight = 0;
  minBestHeight = 1000;
  int count = 0;
  int verPrevLen = 1000;
  int verStartHigh = rand() % scanStepHigh;
  for (int x = verStartHigh; x < getImageWidth(); x = x + scanStepHigh) {
    //! Field scan parameters
    int verStartF = -1;
    int verEndF = -1;
    otherF = 0, greenF = 0;
    fieldMin = getImageHeight();
    fieldMax = 0;
    fieldFound = false;
    //! Ball scan parameters
    int verStartBall = -1;
    otherB, blackB = 0, whiteB = 0;
    //int verStartLine = -1;
    //! Robot scan parameters
    int verStartRobot = -1;
    int whiteR = 0, otherR = 0, greenR = 0;
    //! Team jersey scan parameters
    int verStartJerseyOurs = -1;
    //! Opponent jersey scan parameters
    int verStartJerseyOpps = -1;
    int verStartLow = rand() % scanStepLow;
    for (int y = verStartLow; y < getImageHeight(); y = y + scanStepLow) {
      bool scanInField = y > lastHeight;
      auto color = getYUV(x, y);
      //! Vertical Scanning
      if (colorHandler->isColor(color, Colors::GREEN)) {
        if (scanInField) {
          //! Field Scanning
          if (verStartF == -1) {
            verStartF = y;
          }
          otherF = 0;
          greenF++;

          //! Robot Scanning
          greenR++;
          if (greenR > 1) {
            float r;
            if (otherR != 0) {
              r = whiteR / (float) otherR;
              if (verStartRobot != -1 && otherR > 0 && whiteR > 0 && r > 0.05f && r < 0.99f) {
                auto sl = boost::make_shared < ScannedLine > (Point(
                  x,
                  verStartRobot), Point(x, y - scanStepLow));
                verRobotLines.push_back(sl);
                //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(0,0,255), 1);
                // imshow("image", bgrMat[currentImage]);
                //waitKey(0);
              }
              otherR = 0;
              whiteR = 0;
              greenR = 0;
              verStartRobot = -1;
            }
            verStartRobot = -1;
          }

          //! Ball Scanning
          /*if (scanBall) {
           if (otherB != 0) {
           float rBWhite = whiteB / (float) otherB;
           float rBBlack = blackB / (float) otherB;
           bool check = rBWhite > 0.05 && rBWhite < 0.95f;
           check = check && rBBlack > 0.05 && rBBlack < 0.95f;
           if (
           verStartBall != -1 &&
           check &&
           y - verStartBall < 100) 
           {
           auto sl = 
           boost::make_shared<ScannedLine>(
           Point(x, verStartBall), Point(x, y - scanStepLow)
           );
           verBallLines.push_back(sl);
           //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(0,0,255), 1);
           //imshow("image", bgrMat[currentImage]);
           //waitKey(0);
           }
           otherB = 0;
           whiteB = 0;
           verStartBall = -1;
           } else {
           otherB = 0;
           whiteB = 0;
           verStartBall = -1;
           }
           }*/
        }

        //! Team Jersey Scanning
        if (verStartJerseyOurs != -1) {
          auto sl = boost::make_shared < ScannedLine > (Point(
            x,
            verStartJerseyOurs), Point(x, y - scanStepLow));
          verJerseyLinesOurs.push_back(sl);
          //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,255,0), 2);
          verStartJerseyOurs = -1;
        }

        //! Opponent Jersey Scanning
        if (verStartJerseyOpps != -1) {
          auto sl = boost::make_shared < ScannedLine > (Point(
            x,
            verStartJerseyOpps), Point(x, y - scanStepLow));
          verJerseyLinesOpps.push_back(sl);
          //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,0,0), 2);
          verStartJerseyOpps = -1;
        }
      } else {
        bool isOurs = colorHandler->isColor(color, ourColor);
        bool isOpps = colorHandler->isColor(color, oppColor);

        if (scanInField) {
          bool isWhite = colorHandler->isColor(color, Colors::WHITE);
          bool isBlack = colorHandler->isColor(color, Colors::BLACK);
          //! Field Scanning
          if (verStartF != -1) otherF++;

          /*if (scanBall) {
           //! Ball Scanning
           if (verStartBall == -1) {
           verStartBall = y;
           } else {
           otherB++;
           if (isWhite) {
           whiteB++;
           }
           if (isBlack) {
           blackB++;
           }
           }
           }*/

          //! Robot Scanning
          if (verStartRobot == -1) {
            verStartRobot = y;
          } else {
            otherR++;
            if (isWhite) {
              whiteR++;
            }
          }
          greenR = 0;
        }

        //! Team Jersey Scanning
        if (verStartJerseyOurs == -1) {
          if (isOurs) verStartJerseyOurs = y;
        }

        if (verStartJerseyOpps == -1) {
          if (isOpps) verStartJerseyOpps = y;
        }
        //! Team Jersey Scanning
        if (!isOurs) {
          if (verStartJerseyOurs != -1) {
            auto sl = boost::make_shared < ScannedLine > (Point(
              x,
              verStartJerseyOurs), Point(x, y - scanStepLow));
            verJerseyLinesOurs.push_back(sl);
            //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,255,0), 2);
            verStartJerseyOurs = -1;
          }
        }

        //! Opponent Jersey Scanning
        if (!isOpps) {
          if (verStartJerseyOpps != -1) {
            auto sl = boost::make_shared < ScannedLine > (Point(
              x,
              verStartJerseyOpps), Point(x, y - scanStepLow));
            verJerseyLinesOpps.push_back(sl);
            //line(bgrMat[currentImage], sl->p1, sl->p2, Scalar(255,0,0), 2);
            verStartJerseyOpps = -1;
          }
        }
      }

      if (scanInField) {
        //! Field Scanning
        if (otherF > 2 || y >= vScanLimitIdx) {
          if (verStartF != -1) {
            if (greenF > 2) {
              if (abs(verStartF - verEndF) > verPrevLen) {
                verEndF = y;
                fieldMin = verStartF;
                fieldMax = verEndF;
                fieldFound = true;
                verPrevLen = verEndF - verStartF;
              } else {
                verEndF = y;
                fieldMin = verStartF < fieldMin ? verStartF : fieldMin;
                fieldMax = verEndF > fieldMax ? verEndF : fieldMax;
                fieldFound = true;
                verPrevLen = verEndF - verStartF;
              }
              verStartF = -1;
              greenF = 0;
            } else {
              verStartF = -1;
              greenF = 0;
            }
          }
          verStartF = -1;
          greenF = 0;
        }
      }

      /*if(colorHandler->isColor(color, Colors::WHITE))      
       { 
       if (verStartLine == -1) {
       verStartLine = y; 
       }
       
       if (verStartGoal == -1) {
       verStartGoal = y;
       }
       
       } else {
       if (verStartLine != -1) {
       float diff = y - verStartLine;
       if( diff >= 1 ) {
       int index;
       if (diff == 0) {
       index = verStartLine;
       } else {
       index = (verStartLine + y - scanStepLow) / 2;
       }
       uchar* p = whiteEdges.ptr<uchar>(index);
       p[x] = 255;
       auto se = 
       boost::make_shared<ScannedEdge>(Point(x, index));
       lineEdges.push_back(se);
       //uchar* p = whiteEdges.ptr<uchar>(y - scanStepLow);
       //p[x] = 255;
       //p = whiteEdges.ptr<uchar>(verStartLine);
       //p[x] = 255;
       //whiteEdges.at<uchar>(sl->p1.y, sl->p1.x) = 255;
       //whiteEdges.at<uchar>(sl->p2.y, sl->p2.x) = 255;
       //line(yuv, sl->p1, sl->p2, Scalar(255,0,0), 1);
       
       //auto se2 = boost::make_shared<ScannedEdge>(Point(x, y));              
       //se1->connectedTo = se2;
       //se2->connectedTo = se1;
       //verLineEdges.push_back(se1);
       //verLineEdges.push_back(se2);
       //drawPoint(Point(horStartLine, i), out, Scalar(0,0,255));
       //drawPoint(Point(tmpEndLine, i), out, Scalar(255,0,0));
       //line(yuv, Point(x, verStartLine), Point(x, y), Scalar(255,255,255), 1);
       //imshow("test", out);
       //waitKey(0);
       }
       }
       verStartLine = -1;
       
       if (verStartGoal != -1) {
       float width = y - verStartGoal;
       if( width >= 5 ) {
       auto sl = 
       boost::make_shared<ScannedLine>(
       Point(x, verStartGoal), Point(x, y)
       );
       verGoalLines.push_back(sl);
       //line(yuv, sl->p1, sl->p2, Scalar(255,0,255), 1);
       }
       }   
       verStartGoal = -1;
       }*/
    }

    //! Field points accumulation
    if (fieldFound) {
      borderPoints.push_back(Point(x, fieldMin));
      borderPoints.push_back(Point(x, fieldMax));
      //line(bgrMat[currentImage], Point(x, fieldMin), Point(x, fieldMax), Scalar(0,255,0), 1);
    }

    minBestHeight = fieldMin < minBestHeight ? fieldMin : minBestHeight;
    avgHeight += fieldMin;
    count++;
  }
  avgHeight /= count;
  /*Mat bgr = bgrMat[currentImage];
   for (int i = 0; i < verRobotLines.size(); ++i) {
   if (verRobotLines[i])
   line(bgr, verRobotLines[i]->p1, verRobotLines[i]->p2, Scalar(255,0,0), 1);
   }

   for (int i = 0; i < horRobotLines.size(); ++i) {
   if (horRobotLines[i])
   line(bgr, horRobotLines[i]->p1, horRobotLines[i]->p2, Scalar(255,0,0), 1);
   }*/
#ifdef DEBUG_BUILD
  auto tEnd = high_resolution_clock::now();
  duration<double> timeSpan = tEnd - tStart;
  //LOG_INFO("RegionSegmentation.Update.Time: " << timeSpan.count() << " seconds.");
#endif
  //Mat bgr = bgrMat[currentImage];

  /* for (int i = 0; i < horBallLines.size(); ++i) {
   if (horBallLines[i])
   line(bgr, horBallLines[i]->p1, horBallLines[i]->p2, Scalar(0,255,0), 1);
   }
   
   for (int i = 0; i < verBallLines.size(); ++i) {
   if (verBallLines[i])
   line(bgr, verBallLines[i]->p1, verBallLines[i]->p2, Scalar(0,255,0), 1);
   }*/
  // imshow("bgr", bgr);
  // waitKey(0);
  /*#ifdef DEBUG_BUILD	
   if (GET_DVAR(int, drawRegions)) {
   if(!borderPoints.empty()) {  
   Mat bgr = bgrMat[currentImage];
   
   for (int i = 0; i < verRobotLines.size(); ++i) {
   if (verRobotLines[i])
   line(bgr, verRobotLines[i]->p1, verRobotLines[i]->p2, Scalar(255,0,0), 1);
   }
   
   for (int i = 0; i < horRobotLines.size(); ++i) {
   if (horRobotLines[i])
   line(bgr, horRobotLines[i]->p1, horRobotLines[i]->p2, Scalar(255,0,0), 1);
   }

   for (int i = 0; i < horBallLines.size(); ++i) {
   if (horBallLines[i])
   line(bgr, horBallLines[i]->p1, horBallLines[i]->p2, Scalar(0,255,0), 1);
   }
   
   for (int i = 0; i < verBallLines.size(); ++i) {
   if (verBallLines[i])
   line(bgr, verBallLines[i]->p1, verBallLines[i]->p2, Scalar(0,255,0), 1);
   }
   
   for (int i = 0; i < verGoalLines.size(); ++i) {
   line(bgr, verGoalLines[i]->p1, verGoalLines[i]->p2, Scalar(255,255,255), 1);
   }
   }
   }
   
   if (GET_DVAR(int, sendTime)) {
   auto tEnd = high_resolution_clock::now();
   timeSpan = tEnd - tStart; 
   CommModule::addToLogMsgQueue("RegionSegmentation time: " +
   DataUtils::varToString(timeSpan.count()) + " seconds.");
   }
   #endif*/
}
