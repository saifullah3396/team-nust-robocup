/**
 * @file FeatureExtraction/RANSACEllipseFit.cpp
 *
 * This file defines the class RANSACEllipseFit
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#include "VisionModule/include/FeatureExtraction/RANSACEllipseFit.h"
#include "VisionModule/include/FeatureExtraction/LinesExtraction.h"

bool
RANSACEllipseFit::findBestEllipse(Ellipse& bestEllipse,
  vector<Point2f>& bestInliers)
{
  vector < Point2f > bestOverallInlierSet;
  int bestOverallScore = 0;
  for (int i = 0; i < pointVectors.size(); ++i) {
    vector < Point2f > points = pointVectors[i];
    if (points.size() < params.minPoints) continue;
    int minInliers = points.size() * 0.35;
    Ellipse ellipse(imageWidth, imageHeight);
    int bestInlierScore = 0;
    vector < Point2f > bestInlierSet;
    //auto tStart = high_resolution_clock::now();
    for (int j = 0; j < params.iter; ++j) {
      //! Choose points at random
      //auto tStart = high_resolution_clock::now();
      vector < Point2f > consensusSet;
      chooseRandom(points, consensusSet);
      //auto tEnd = high_resolution_clock::now();
      //duration<double> timeSpan = tEnd - tStart;
      //if (j == 0) cout << "Choose Random time: " << timeSpan.count() << " seconds." << endl;
      //! Fit ellipse to the points in consensus set
      //auto tStart2 = high_resolution_clock::now();
      VectorXd ptsX, ptsY;
      ptsX.resize(consensusSet.size());
      ptsY.resize(consensusSet.size());
      for (int k = 0; k < consensusSet.size(); ++k) {
        ptsX[k] = consensusSet[k].x;
        ptsY[k] = consensusSet[k].y;
      }
      DirectEllipseFit<double> ellipFit(ptsX, ptsY, imageWidth, imageHeight);
      Ellipse ellipseTemp = ellipFit.doEllipseFit();
      //auto tEnd2 = high_resolution_clock::now();
      //duration<double> timeSpan2 = tEnd2 - tStart2;
      //if (j == 0) cout << "Ellipse Fit time: " << timeSpan2.count() << " seconds." << endl;
      //! Check for inliers using the remaining points
      //auto tStart3 = high_resolution_clock::now();
      vector < Point2f > inliersSet;
      distance(ellipseTemp, points, inliersSet);
      //auto tEnd3 = high_resolution_clock::now();
      //duration<double> timeSpan3 = tEnd3 - tStart3;
      //if (j == 0) cout << "Distance time: " << timeSpan3.count() << " seconds." << endl;
      unsigned inliers = inliersSet.size();
      //! Find the random set with the most number of inliers
      if (inliers > minInliers && inliers > bestInlierScore) {
        ellipse = ellipseTemp;
        bestInlierScore = inliers;
        bestInlierSet = inliersSet;
      }
    }
    //auto tEnd = high_resolution_clock::now();
    //duration<double> timeSpan = tEnd - tStart;
    //if (i == 0) cout << "Iter: " << timeSpan.count() << " seconds." << endl;
    //cout << "bestInlierSet: " << bestInlierSet.size() << endl;
    //cout << "bestInlierScore: " << bestInlierScore << endl;
    //cout << "bestOverallScore: " << bestOverallScore << endl;
    //cout << "ellipse.isGood(): " << ellipse.isGood() << endl;
    //! Find the points vector with ellipse that has the most number of inliers
    if (bestInlierScore > bestOverallScore && LinesExtraction::checkEllipse(
      ellipse)) {
      bestOverallScore = bestInlierScore;
      bestOverallInlierSet = bestInlierSet;
    }
  }

  //cout << "bestOverallScore: " << bestOverallScore << endl;
  //cout << "bestOverallInlierSet: " << bestOverallInlierSet.size() << endl;
  if (!bestOverallInlierSet.empty()) {
    VectorXd ptsX, ptsY;
    ptsX.resize(bestOverallInlierSet.size());
    ptsY.resize(bestOverallInlierSet.size());
    for (int i = 0; i < bestOverallInlierSet.size(); ++i) {
      ptsX[i] = bestOverallInlierSet[i].x;
      ptsY[i] = bestOverallInlierSet[i].y;
    }
    DirectEllipseFit<double> ellipFit(ptsX, ptsY, imageWidth, imageHeight);
    bestEllipse = ellipFit.doEllipseFit();
    if (bestEllipse.conic[5] < 0) {
      for (int i = 0; i < bestEllipse.conic.size(); ++i)
        bestEllipse.conic[i] *= -1.f;
    }
  }
  bestInliers = bestOverallInlierSet;
  return LinesExtraction::checkEllipse(bestEllipse);
}

bool
RANSACEllipseFit::directFit(Ellipse& ellipse, const vector<Point2f>& points)
{
  int n = points.size();
  Mat designMat;
  for (int i = 0; i < n; i++) {
    Point2f p = points[i];
    Mat r(1, 6, CV_32FC1);
    r =
      (Mat_<float>(1, 6) << (p.x) * (p.x), (p.x) * (p.y), (p.y) * (p.y), p.x, p.y, 1.f);
    designMat.push_back(r);
  }
  Mat scatterMat = designMat.t() * designMat, scatterMatInv;
  double d = invert(scatterMat, scatterMatInv);
  if (d < 0.001) return false;

  Mat constraintMat = Mat::zeros(6, 6, CV_32F);
  constraintMat.at<float>(2, 0) = 2;
  constraintMat.at<float>(1, 1) = -1;
  constraintMat.at<float>(0, 2) = 2;

  //! Using Eigen to calculate eigenvalues and eigenvectors
  Mat prod = scatterMatInv * constraintMat;
  Eigen::MatrixXd prodE;
  cv2eigen(prod, prodE);
  EigenSolver<MatrixXd> es(prodE);
  Mat evec, eval, vec(6, 6, CV_32FC1), val(6, 1, CV_32FC1);
  eigen2cv(es.eigenvectors(), evec);
  eigen2cv(es.eigenvalues(), eval);
  evec.convertTo(evec, CV_32F);
  eval.convertTo(eval, CV_32F);
  //! Eigen returns complex parts in the second channel 
  //! (which are all 0 here) so select just the first channel
  int fromTo[] =
    { 0, 0 };
  mixChannels(&evec, 1, &vec, 1, fromTo, 1);
  mixChannels(&eval, 1, &val, 1, fromTo, 1);

  Point maxLoc;
  minMaxLoc(val, NULL, NULL, NULL, &maxLoc);
  Mat conic = vec.col(maxLoc.y);
  ellipse = Ellipse(imageWidth, imageHeight);
  ellipse.conic[0] = conic.at<float>(0, 0);
  ellipse.conic[1] = conic.at<float>(1, 0);
  ellipse.conic[2] = conic.at<float>(2, 0);
  ellipse.conic[3] = conic.at<float>(3, 0);
  ellipse.conic[4] = conic.at<float>(4, 0);
  ellipse.conic[5] = conic.at<float>(5, 0);
  ellipse.alge2geom();
}

void
RANSACEllipseFit::chooseRandom(vector<Point2f>& points,
  vector<Point2f>& consensusSet)
{
  consensusSet.clear();
  //! Randomly shuffle all elements of the given points vector
  std::random_shuffle(points.begin(), points.end());
  //! Put the first N elements into the consensus set and
  //! the rest in remains to check for inliers
  consensusSet.push_back(points[0]);
  for (int i = 0; i < params.n; ++i) {
    if (i < params.n) consensusSet.push_back(points[i]);
  }
}

void
RANSACEllipseFit::distance(Ellipse& e, const vector<Point2f>& points,
  vector<Point2f>& inliersSet)
{
  //float cosa = cos(M_PI - e.phi);
  //float sina = sin(M_PI - e.phi);
  float cosa = cos(M_PI - e.phi);
  float sina = sin(M_PI - e.phi);
  for (int i = params.n; i < points.size(); ++i) {
    float xc = points[i].x - e.cx;
    float yc = points[i].y - e.cy;
    float xct = xc * cosa - yc * sina;
    float yct = xc * sina + yc * cosa;
    float px = xct / e.rl;
    float py = yct / e.rs;
    float dist = px * px + py * py;
    if (dist > 1.1f || dist < 0.9f) continue;
    inliersSet.push_back(points[i]);
  }
}
