/**
 * @file FeatureExtraction/DirectEllipseFit.h
 *
 * This file defines the class DirectEllipseFit
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include "VisionModule/include/FeatureExtraction/Ellipse.h"
#include "Utils/include/MathsUtils.h"

using namespace std;
using namespace cv;
using namespace Utils;

extern "C" void
dggev_(const char* JOBVL, const char* JOBVR, const int* N, const double* A,
  const int* LDA, const double* B, const int* LDB, double* ALPHAR,
  double* ALPHAI, double* BETA, double* VL, const int* LDVL, double* VR,
  const int* LDVR, double* WORK, const int* LWORK, int* INFO);

template<typename T>
  class DirectEllipseFit
  {
  public:
    DirectEllipseFit(const Matrix<T, Dynamic, 1> &xData,
      const Matrix<T, Dynamic, 1> &yData, const int& imageWidth,
      const int& imageHeight);
    Ellipse
    doEllipseFit();

  private:
    T
    getMeanValue(const Matrix<T, Dynamic, 1> &data);
    T
    getMaxValue(const Matrix<T, Dynamic, 1> &data);
    T
    getMinValue(const Matrix<T, Dynamic, 1> &data);
    T
    getScaleValue(const Matrix<T, Dynamic, 1> &data);
    Matrix<T, Dynamic, 1>
    symmetricNormalize(const Matrix<T, Dynamic, 1> &data);
    //Make sure xData and yData are of same size
    Matrix<T, Dynamic, 1>
    dotMultiply(const Matrix<T, Dynamic, 1> &xData,
      const Matrix<T, Dynamic, 1> &yData);
    //Get n*6 design matrix D, make sure xData and yData are of same size
    Matrix<T, Dynamic, 6>
    getDesignMatrix(const Matrix<T, Dynamic, 1> &xData,
      const Matrix<T, Dynamic, 1> &yData);
    //Get 6*6 constraint matrix C
    Matrix<T, 6, 6>
    getConstraintMatrix();
    //Get 6*6 scatter matrix S from design matrix
    Matrix<T, 6, 6>
    getScatterMatrix(const Matrix<T, Dynamic, 6> &dMtrx);

    /**
     * @brief solveGeneralEigens:   Solve generalized eigensystem
     * @note        For real eiginsystem solving.
     * @param sMtrx:    6*6 square matrix in this application
     * @param cMtrx:    6*6 square matrix in this application
     * @param eigVV:    eigenvalues and eigenvectors, 6*7 matrix
     * @return  success or failure status
     */
    bool
    solveGeneralEigens(const Matrix<T, 6, 6> &sMtrx,
      const Matrix<T, 6, 6> &cMtrx, Matrix<T, Dynamic, Dynamic> &eigVV);
    //Convert matrix expression from nested QVector to 1-order array
    double *
    mtrx2array(const Matrix<T, Dynamic, Dynamic> &mtrx);

    /**
     * @brief calcEllipsePara:  calculate ellipse parameter form eigen information
     * @param eigVV:    eigenvalues and eigenvectors
     * @return ellipse parameter
     */
    Ellipse
    calcEllipsePara(const Matrix<T, Dynamic, Dynamic> &eigVV);

  private:
    Matrix<T, Dynamic, 1> m_xData, m_yData;
    int imageWidth, imageHeight;
  };

/*******************************************************************************
 * Template Class Defination
 ******************************************************************************/
template<typename T>
  DirectEllipseFit<T>::DirectEllipseFit(const Matrix<T, Dynamic, 1> &xData,
    const Matrix<T, Dynamic, 1> &yData, const int& imageWidth,
    const int& imageHeight)
  {
    m_xData = xData;
    m_yData = yData;
    this->imageWidth = imageWidth;
    this->imageHeight = imageHeight;
  }

template<typename T>
  Ellipse
  DirectEllipseFit<T>::doEllipseFit()
  {
    //Data preparation: normalize data
    Matrix<T, Dynamic, 1> xData = symmetricNormalize(m_xData);
    Matrix<T, Dynamic, 1> yData = symmetricNormalize(m_yData);

    //Bulid n*6 design matrix, n is size of xData or yData
    Matrix<T, Dynamic, 6> dMtrx = getDesignMatrix(xData, yData);

    //Bulid 6*6 scatter matrix
    Matrix<T, 6, 6> sMtrx = getScatterMatrix(dMtrx);

    //Build 6*6 constraint matrix
    Matrix<T, 6, 6> cMtrx = getConstraintMatrix();

    //Solve eigensystem
    Matrix<T, Dynamic, Dynamic> eigVV;
    bool flag = solveGeneralEigens(sMtrx, cMtrx, eigVV);
    if (!flag) cout << "Eigensystem solving failure!";

    Ellipse ellip = calcEllipsePara(eigVV);

    return ellip;
  }

template<typename T>
  T
  DirectEllipseFit<T>::getMeanValue(const Matrix<T, Dynamic, 1> &data)
  {
    T mean = 0;
    for (int i = 0; i < data.size(); ++i)
      mean += data[i];

    return mean / data.size();
  }

template<typename T>
  T
  DirectEllipseFit<T>::getMaxValue(const Matrix<T, Dynamic, 1> &data)
  {
    T max = data[0];
    for (int i = 1; i < data.size(); ++i)
      if (data[i] > max) max = data[i];

    return max;
  }

template<typename T>
  T
  DirectEllipseFit<T>::getMinValue(const Matrix<T, Dynamic, 1> &data)
  {
    T min = data[0];
    for (int i = 1; i < data.size(); ++i)
      if (data[i] < min) min = data[i];

    return min;
  }

template<typename T>
  T
  DirectEllipseFit<T>::getScaleValue(const Matrix<T, Dynamic, 1> &data)
  {
    return (0.5 * (getMaxValue(data) - getMinValue(data)));
  }

template<typename T>
  Matrix<T, Dynamic, 1>
  DirectEllipseFit<T>::symmetricNormalize(const Matrix<T, Dynamic, 1> &data)
  {
    T mean = getMeanValue(data);
    T normScale = getScaleValue(data);

    Matrix<T, Dynamic, 1> symData;
    symData.resize(data.size());
    for (int i = 0; i < data.size(); ++i)
      symData[i] = ((data[i] - mean) / normScale);

    return symData;
  }

template<typename T>
  Matrix<T, Dynamic, 1>
  DirectEllipseFit<T>::dotMultiply(const Matrix<T, Dynamic, 1> &xData,
    const Matrix<T, Dynamic, 1> &yData)
  {
    Matrix<T, Dynamic, 1> product;
    product.resize(xData.size());
    for (int i = 0; i < xData.size(); ++i)
      product[i] = (xData[i] * yData[i]);

    return product;
  }

template<typename T>
  Matrix<T, Dynamic, 6>
  DirectEllipseFit<T>::getDesignMatrix(const Matrix<T, Dynamic, 1> &xData,
    const Matrix<T, Dynamic, 1> &yData)
  {
    Matrix<T, Dynamic, 6> designMtrx;
    designMtrx.resize(xData.size(), 6);
    Matrix<T, Dynamic, 1> oneVec = Matrix<T, Dynamic, Dynamic>::Ones(
      xData.size(),
      1);
    designMtrx.block(0, 0, xData.size(), 1) = dotMultiply(xData, xData);
    designMtrx.block(0, 1, xData.size(), 1) = dotMultiply(xData, yData);
    designMtrx.block(0, 2, xData.size(), 1) = dotMultiply(yData, yData);
    designMtrx.block(0, 3, xData.size(), 1) = xData;
    designMtrx.block(0, 4, xData.size(), 1) = yData;
    designMtrx.block(0, 5, xData.size(), 1) = oneVec;

    return designMtrx;
  }

template<typename T>
  Matrix<T, 6, 6>
  DirectEllipseFit<T>::getConstraintMatrix()
  {
    Matrix<T, 6, 1> sglVec = Matrix<T, 6, 1>::Zero();
    Matrix<T, 6, 6> consMtrx;
    consMtrx << sglVec, sglVec, sglVec, sglVec, sglVec, sglVec;

    consMtrx(1, 1) = 1;
    consMtrx(0, 2) = -2;
    consMtrx(2, 0) = -2;

    return consMtrx;
  }

template<typename T>
  Matrix<T, 6, 6>
  DirectEllipseFit<T>::getScatterMatrix(const Matrix<T, Dynamic, 6> &dMtrx)
  {
    Matrix<T, 6, Dynamic> tMtrx = dMtrx.transpose();
    return (tMtrx * dMtrx);
  }

template<typename T>
  bool
  DirectEllipseFit<T>::solveGeneralEigens(const Matrix<T, 6, 6> &sMtrx,
    const Matrix<T, 6, 6> &cMtrx, Matrix<T, Dynamic, Dynamic> &eigVV)
  {
    //Parameter initialization
    char jobvl = 'N';
    char jobvr = 'V';
    int nOrder = sMtrx.rows();
    double *sArray = mtrx2array(sMtrx);
    double *cArray = mtrx2array(cMtrx);
    double *alphaR = new double[nOrder];
    double *alphaI = new double[nOrder];
    double *beta = new double[nOrder];
    double *VL = new double[nOrder * nOrder];
    double *VR = new double[nOrder * nOrder];
    int lwork = 8 * nOrder;
    double *work = new double[lwork];
    int info;

    //Solve generalized eigensystem
    dggev_(
      &jobvl,
      &jobvr,
      &nOrder,
      sArray,
      &nOrder,
      cArray,
      &nOrder,
      alphaR,
      alphaI,
      beta,
      VL,
      &nOrder,
      VR,
      &nOrder,
      work,
      &lwork,
      &info);

    //Output eigenvalues and eigenvectors
    eigVV = Matrix<T, Dynamic, Dynamic>();
    eigVV.resize(nOrder, nOrder + 1);
    for (int i = 0; i < nOrder; ++i) {
      Matrix<T, Dynamic, 1> tmpVec;
      tmpVec.resize(nOrder + 1, 1);
      tmpVec[0] = alphaR[i] / beta[i];
      for (int j = 0; j < nOrder; ++j) {
        tmpVec[j + 1] = VR[i * nOrder + j];
      }
      eigVV.block(i, 0, 1, tmpVec.size()) = tmpVec.transpose();
    }
    //Free memory
    delete[] sArray;
    delete[] cArray;
    delete[] alphaR;
    delete[] alphaI;
    delete[] beta;
    delete[] VL;
    delete[] VR;
    delete[] work;

    //output calculation status
    if (info == 0) return true;
    else return false;
  }

template<typename T>
  double *
  DirectEllipseFit<T>::mtrx2array(const Matrix<T, Dynamic, Dynamic> &mtrx)
  {
    int nRow = mtrx.rows();
    int nCol = mtrx.cols();
    double *array = new double[nRow * nCol];
    memset(array, 0, nRow * nCol * sizeof(double));

    for (int i = 0; i < nRow; ++i) {
      for (int j = 0; j < nCol; ++j) {
        array[i * nCol + j] = mtrx(j, i);
      }
    }

    return array;
  }

template<typename T>
  Ellipse
  DirectEllipseFit<T>::calcEllipsePara(const Matrix<T, Dynamic, Dynamic> &eigVV)
  {
    //Extract eigenvector corresponding to negative eigenvalue
    int eigIdx = -1;
    for (int i = 0; i < eigVV.rows(); ++i) {
      T tmpV = eigVV(i, 0);
      if (tmpV < 1e-6 && !std::isinf(tmpV)) {
        eigIdx = i;
        break;
      }
    }
    if (eigIdx < 0) return Ellipse(imageWidth, imageHeight);

    //Unnormalize and get coefficients of conic section
    T tA = eigVV(eigIdx, 1);
    T tB = eigVV(eigIdx, 2);
    T tC = eigVV(eigIdx, 3);
    T tD = eigVV(eigIdx, 4);
    T tE = eigVV(eigIdx, 5);
    T tF = eigVV(eigIdx, 6);

    T mx = getMeanValue(m_xData);
    T my = getMeanValue(m_yData);
    T sx = getScaleValue(m_xData);
    T sy = getScaleValue(m_yData);

    Ellipse ellip(imageWidth, imageHeight);
    ellip.conic[0] = tA * sy * sy;
    ellip.conic[1] = tB * sx * sy;
    ellip.conic[2] = tC * sx * sx;
    ellip.conic[3] =
      -2 * tA * sy * sy * mx - tB * sx * sy * my + tD * sx * sy * sy;
    ellip.conic[4] =
      -tB * sx * sy * mx - 2 * tC * sx * sx * my + tE * sx * sx * sy;
    ellip.conic[5] =
      tA * sy * sy * mx * mx + tB * sx * sy * mx * my + tC * sx * sx * my * my - tD * sx * sy * sy * mx - tE * sx * sx * sy * my + tF * sx * sx * sy * sy;
    ellip.alge2geom();
    return ellip;
  }

template class DirectEllipseFit<double> ;
