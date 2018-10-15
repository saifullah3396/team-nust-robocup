/**
 * @file ColorHandler/ColorHandler.h
 *
 * This file declares the class ColorHandler.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 23 Nov 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/VisionUtils.h"
#include "VisionModule/include/VisionModule.h"

using namespace Utils;

enum Colors
{
  WHITE = 0,
  BLACK,
  GREEN,
  BLUE,
  RED,
  YELLOW,
  NUM_COLORS
};

struct TNColor
{
  TNColor()
  {
  }

  TNColor(const uint8_t& y, const uint8_t& u, const uint8_t& v) :
    y(y), u(u), v(v)
  {
  }

  bool
  operator<(const TNColor& rhs) const
  {
    return this->y <= rhs.y && this->u <= rhs.u && this->v < rhs.v;
  }

  bool
  operator>(const TNColor& rhs) const
  {
    return this->y >= rhs.y && this->u >= rhs.u && this->v > rhs.v;
  }

  uint8_t y;
  uint8_t u;
  uint8_t v;
};

class ColorHandler
{
public:
  /**
   * Default constructor for this class.
   *
   * @param visionModule: Pointer to parent VisionModule
   */
  ColorHandler() :
    colorRangesU(NUM_COLORS), colorRangesL(NUM_COLORS), colorLutY(NUM_COLORS),
      colorLutU(NUM_COLORS), colorLutV(NUM_COLORS), nTables(NUM_COLORS),
      prevColorIndex(0)
  {
    GET_CONFIG(
      "ColorConfigYuv",
      (int, White.tables, nTables[WHITE]), (int, Black.tables, nTables[BLACK]), (int, Green.tables, nTables[GREEN]), (int, Red.tables, nTables[RED]), (int, Blue.tables, nTables[BLUE]), (int, Yellow.tables, nTables[YELLOW]), )

    for (int i = 0; i < NUM_COLORS; ++i) {
      colorRangesU[i].resize(nTables[i]);
      colorRangesL[i].resize(nTables[i]);
      //cout << colorRangesU[i].size() << endl;
    }
    GET_CONFIG(
      "ColorConfigYuv",
      (int, WhiteLower1.y, colorRangesL[WHITE][0].y), (int, WhiteLower1.u, colorRangesL[WHITE][0].u), (int, WhiteLower1.v, colorRangesL[WHITE][0].v), (int, WhiteUpper1.y, colorRangesU[WHITE][0].y), (int, WhiteUpper1.u, colorRangesU[WHITE][0].u), (int, WhiteUpper1.v, colorRangesU[WHITE][0].v), (int, WhiteLower2.y, colorRangesL[WHITE][1].y), (int, WhiteLower2.u, colorRangesL[WHITE][1].u), (int, WhiteLower2.v, colorRangesL[WHITE][1].v), (int, WhiteUpper2.y, colorRangesU[WHITE][1].y), (int, WhiteUpper2.u, colorRangesU[WHITE][1].u), (int, WhiteUpper2.v, colorRangesU[WHITE][1].v), (int, BlackLower.y, colorRangesL[BLACK][0].y), (int, BlackLower.u, colorRangesL[BLACK][0].u), (int, BlackLower.v, colorRangesL[BLACK][0].v), (int, BlackUpper.y, colorRangesU[BLACK][0].y), (int, BlackUpper.u, colorRangesU[BLACK][0].u), (int, BlackUpper.v, colorRangesU[BLACK][0].v), (int, GreenLower1.y, colorRangesL[GREEN][0].y), (int, GreenLower1.u, colorRangesL[GREEN][0].u), (int, GreenLower1.v, colorRangesL[GREEN][0].v), (int, GreenUpper1.y, colorRangesU[GREEN][0].y), (int, GreenUpper1.u, colorRangesU[GREEN][0].u), (int, GreenUpper1.v, colorRangesU[GREEN][0].v), (int, GreenLower2.y, colorRangesL[GREEN][1].y), (int, GreenLower2.u, colorRangesL[GREEN][1].u), (int, GreenLower2.v, colorRangesL[GREEN][1].v), (int, GreenUpper2.y, colorRangesU[GREEN][1].y), (int, GreenUpper2.u, colorRangesU[GREEN][1].u), (int, GreenUpper2.v, colorRangesU[GREEN][1].v), (int, GreenLower3.y, colorRangesL[GREEN][2].y), (int, GreenLower3.u, colorRangesL[GREEN][2].u), (int, GreenLower3.v, colorRangesL[GREEN][2].v), (int, GreenUpper3.y, colorRangesU[GREEN][2].y), (int, GreenUpper3.u, colorRangesU[GREEN][2].u), (int, GreenUpper3.v, colorRangesU[GREEN][2].v), (int, BlueLower.y, colorRangesL[BLUE][0].y), (int, BlueLower.u, colorRangesL[BLUE][0].u), (int, BlueLower.v, colorRangesL[BLUE][0].v), (int, BlueUpper.y, colorRangesU[BLUE][0].y), (int, BlueUpper.u, colorRangesU[BLUE][0].u), (int, BlueUpper.v, colorRangesU[BLUE][0].v), (int, RedLower.y, colorRangesL[RED][0].y), (int, RedLower.u, colorRangesL[RED][0].u), (int, RedLower.v, colorRangesL[RED][0].v), (int, RedUpper.y, colorRangesU[RED][0].y), (int, RedUpper.u, colorRangesU[RED][0].u), (int, RedUpper.v, colorRangesU[RED][0].v), (int, YellowLower.y, colorRangesL[YELLOW][0].y), (int, YellowLower.u, colorRangesL[YELLOW][0].u), (int, YellowLower.v, colorRangesL[YELLOW][0].v), (int, YellowUpper.y, colorRangesU[YELLOW][0].y), (int, YellowUpper.u, colorRangesU[YELLOW][0].u), (int, YellowUpper.v, colorRangesU[YELLOW][0].v), );
    //cout << "tables" << endl;
    for (int i = 0; i < NUM_COLORS; ++i) {
      for (int j = 0; j < nTables[i]; ++j) {
        colorLutY[i].push_back(new uint8_t[256]);
        colorLutU[i].push_back(new uint8_t[256]);
        colorLutV[i].push_back(new uint8_t[256]);
        createColorLut(
          colorLutY[i].back(),
          colorLutU[i].back(),
          colorLutV[i].back(),
          colorRangesL[i][j],
          colorRangesU[i][j]);
      }
    }

    uHistBuff.set_capacity(5);
    vHistBuff.set_capacity(5);
    pixelsBuff.set_capacity(5);
    greenHistU = new uint8_t[256];
    greenHistV = new uint8_t[256];
    fieldHist = false;
  }

  /**
   * Default destructor for this class.
   */
  ~ColorHandler()
  {
  }

  void
  update()
  {
    /*static int colorIndex = 0;
     static TNColor colorLower, colorUpper;
     string track = "Color Type";
     VisionUtils::createWindow("Color-Calibration");
     VisionUtils::addTrackBar(track, "Color-Calibration", &colorIndex, NUM_COLORS + 1);
     if (colorIndex > 0 && colorIndex != prevColorIndex) {
     colorLower.y = colorRangesL[colorIndex-1].y;
     colorLower.u = colorRangesL[colorIndex-1].u;
     colorLower.v = colorRangesL[colorIndex-1].v;
     colorUpper.y = colorRangesU[colorIndex-1].y;
     colorUpper.u = colorRangesU[colorIndex-1].u;
     colorUpper.v = colorRangesU[colorIndex-1].v;
     }
     if (colorIndex > 0) {
     VisionUtils::addTrackBar("Color-LowerH", "Color-Calibration", &colorLower.y, 255);
     VisionUtils::addTrackBar("Color-LowerS", "Color-Calibration", &colorLower.u, 255);
     VisionUtils::addTrackBar("Color-LowerV", "Color-Calibration", &colorLower.v, 255);
     VisionUtils::addTrackBar("Color-UpperH", "Color-Calibration", &colorUpper.y, 255);
     VisionUtils::addTrackBar("Color-UpperS", "Color-Calibration", &colorUpper.u, 255);
     VisionUtils::addTrackBar("Color-UpperV", "Color-Calibration", &colorUpper.v, 255);
     colorRangesL[colorIndex-1].y = colorLower.y;
     colorRangesL[colorIndex-1].u = colorLower.u;
     colorRangesL[colorIndex-1].v = colorLower.v;
     colorRangesU[colorIndex-1].y = colorUpper.y;
     colorRangesU[colorIndex-1].u = colorUpper.u;
     colorRangesU[colorIndex-1].v = colorUpper.v;
     }
     prevColorIndex = colorIndex;
     */
  }

  inline void
  getBinary(const Mat& in, Mat& out, const unsigned& colorIndex) const
  {
    for (int i = 0; i < nTables[colorIndex]; ++i) {
      Mat binary;
      inRange(
        in,
        Scalar(
          colorRangesL[colorIndex][i].y,
          colorRangesL[colorIndex][i].u,
          colorRangesL[colorIndex][i].v),
        Scalar(
          colorRangesU[colorIndex][i].y,
          colorRangesU[colorIndex][i].u,
          colorRangesU[colorIndex][i].v),
        binary);
      if (out.empty()) out = binary;
      else bitwise_or(out, binary, out);
    }
  }

  void
  computeUVHist(const Mat& uv, const Mat& mask, const bool& drawHists)
  {
    Mat uvCh[2];
    split(uv, uvCh);
    Mat uHist, vHist;
    const float histRange[] =
      { 0, 256 };
    const float* range =
      { histRange };
    calcHist(
      &uvCh[0],
      1,
      0,
      mask,
      uHist,
      1,
      &histBins,
      &range,
      uniformHist,
      accumulateHist);
    calcHist(
      &uvCh[1],
      1,
      0,
      mask,
      vHist,
      1,
      &histBins,
      &range,
      uniformHist,
      accumulateHist);
    uHistBuff.push_back(uHist);
    vHistBuff.push_back(vHist);
    pixelsBuff.push_back(countNonZero(mask));
    //! Draw histograms for debugging
    if (uHistBuff.size() >= 5) {
      Mat avgHistU = uHistBuff[0];
      Mat avgHistV = vHistBuff[0];
      int pixels = pixelsBuff[0];
      for (int i = 1; i < 5; ++i) {
        addWeighted(uHistBuff[i], 1.0, avgHistU, 1.0, 0, avgHistU);
        addWeighted(vHistBuff[i], 1.0, avgHistV, 1.0, 0, avgHistV);
        pixels += pixelsBuff[i];
      }
      avgHistU = avgHistU / pixels;
      avgHistV = avgHistV / pixels;
      //! Remove values below 35% of maximum value.
      double uMin, uMax, vMin, vMax;
      minMaxLoc(avgHistU, &uMin, &uMax);
      minMaxLoc(avgHistV, &vMin, &vMax);
      for (int i = 0; i < 256; ++i) {
        greenHistU[i] = avgHistU.at<float>(i / 8) < 0.3 * uMax ? 0 : 1.0;
        greenHistV[i] = avgHistV.at<float>(i / 8) < 0.3 * vMax ? 0 : 1.0;
      }
      fieldHist = true;
#ifdef DEBUG_BUILD
      if (drawHists) {
        Mat drawHistU = avgHistU;
        Mat drawHistV = avgHistV;
        for (int i = 0; i < 32; ++i) {
          drawHistU.at<float>(i) = drawHistU.at<float>(i) < 0.3 * uMax ? 0 : drawHistU.at<float>(i);
          drawHistV.at<float>(i) = drawHistV.at<float>(i) < 0.3 * vMax ? 0 : drawHistV.at<float>(i);
        }
        int histWidth = 256, histHeight = 400;
        int binWidth = cvRound(double(histWidth/histBins));
        Mat histImage(histHeight, histWidth, CV_8UC3, Scalar(0,0,0));
        //! Drawing all peaks
        for (int i = 0; i < histWidth; ++i) {
          line(
            histImage,
            Point(i, histHeight),
            Point(i, cvRound(histHeight * (1 - drawHistU.at<float>(i/binWidth)))),
            Scalar(255, 0, 0), 2, 8, 0
          );
          line(
            histImage,
            Point(i, histHeight),
            Point(i, cvRound(histHeight * (1 - drawHistV.at<float>(i/binWidth)))),
            Scalar(0, 255, 0), 2, 8, 0
          );
        }
        VisionUtils::displayImage(histImage, "histImage", 1.0);
        waitKey(0);
      }
#endif
    } else {
      fieldHist = false;
    }
  }

  inline bool
  isGreenHist(const TNColor& color) const
  {
    if ((int) greenHistU[color.u] && (int) greenHistV[color.v]) return true;
    return false;
  }

  inline bool
  isColor(const TNColor& color, const unsigned& colorIndex) const
  {
    for (int i = 0; i < nTables[colorIndex]; ++i) {
      if ((int) colorLutY[colorIndex][i][color.y] && (int) colorLutU[colorIndex][i][color.u] && (int) colorLutV[colorIndex][i][color.v]) return true;
    }
    return false;
  }

  bool&
  fieldHistFormed()
  {
    return fieldHist;
  }

private:
  void
  createColorLut(uint8_t*& tableY, uint8_t*& tableU, uint8_t*& tableV,
    const TNColor& min, const TNColor& max)
  {
    tableY = new uint8_t[256];
    tableU = new uint8_t[256];
    tableV = new uint8_t[256];
    int j = 0;
    for (int i = 0; i <= 255; ++i) {
      tableY[i] = i >= min.y && i <= max.y ? 1 : 0;
      tableU[i] = i >= min.u && i <= max.u ? 1 : 0;
      tableV[i] = i >= min.v && i <= max.v ? 1 : 0;
    }
  }

  vector<vector<TNColor> > colorRangesU;
  vector<vector<TNColor> > colorRangesL;
  vector<vector<uint8_t*> > colorLutY;
  vector<vector<uint8_t*> > colorLutU;
  vector<vector<uint8_t*> > colorLutV;
  vector<int> nTables;
  int prevColorIndex;

  //! Histogram settings
  //! Histogram bins
  const int histBins = 32;

  //! Opencv calcHist settings
  const bool uniformHist = true;
  const bool accumulateHist = false;
  bool fieldHist;

  //! Accumulative Histogram buffers
  boost::circular_buffer<Mat> uHistBuff;
  boost::circular_buffer<Mat> vHistBuff;
  boost::circular_buffer<int> pixelsBuff;
  uint8_t* greenHistU;
  uint8_t* greenHistV;
};

typedef boost::shared_ptr<ColorHandler> ColorHandlerPtr;
