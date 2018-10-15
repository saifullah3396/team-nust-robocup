/**
 * @file MotionModule/src/KickModule/KickModule.cpp
 *
 * This file implements the class KickModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017
 */

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "MotionModule/include/KickModule/KickModule.h"
#include "MotionModule/include/KickModule/Types/JSOImpKick.h"
#include "MotionModule/include/KickModule/Types/JSE2DImpKick.h"

float KickModule::ballMass;
float KickModule::ballRadius;
float KickModule::sf;
float KickModule::rf;
BSpline* KickModule::lFootContour;
BSpline* KickModule::rFootContour;
//Matrix<float, 4, 3> KickModule::lLeftBezierContour; 
//Matrix<float, 4, 3> KickModule::lRightBezierContour; 
//Matrix<float, 4, 3> KickModule::rLeftBezierContour; 
//Matrix<float, 4, 3> KickModule::rRightBezierContour;
//vector<Vector3f> KickModule::lLeftBezierConsts;
//vector<Vector3f> KickModule::lRightBezierConsts;
//vector<Vector3f> KickModule::rLeftBezierConsts;
//vector<Vector3f> KickModule::rRightBezierConsts;

boost::shared_ptr<KickModule> KickModule::getType(
  MotionModule* motionModule, const BehaviorConfigPtr& cfg) 
{ 
  KickModule* km;
  switch (cfg->type) {
      case (unsigned) MBKickTypes::JOINT_SPACE_OPT_IMP_KICK: 
        km = new JSOImpKick(motionModule, cfg); break;
      case (unsigned) MBKickTypes::JOINT_SPACE_EST_2D_IMP_KICK: 
        km = new JSE2DImpKick(motionModule, cfg); break;
      default: km = new JSOImpKick(motionModule, cfg); break;
  }
  return boost::shared_ptr<KickModule>(km);
}

MBKickConfigPtr KickModule::getBehaviorCast()
{
  return boost::static_pointer_cast <MBKickConfig> (config);
}

void
KickModule::loadExternalConfig()
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("EnvProperties", 
      (float, ballRadius, ballRadius), 
      (float, ballMass, ballMass), 
      (float, coeffSF, sf), 
      (float, coeffRF, rf),
    )
    lFootContour = new BSpline(ConfigManager::getConfigDirPath() + "left_foot_contour.xml");
    rFootContour = new BSpline(ConfigManager::getConfigDirPath() + "right_foot_contour.xml");
    loaded = true;
  }
}
/*
void
KickModule::setBezierContourConstants()
{
  Matrix4f bezierMat;
  bezierMat << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
  // Left foot constants
	for (int i = 0; i < lLeftBezierContour.rows(); ++i) {
		for (int j = 0; j < lLeftBezierContour.cols(); ++j) {
			// left side of left foot. I know its confusing
			lLeftBezierContour(i, j) = leftFootContour[i+4][j]; 
		}
	}
	// left side of left foot defined by one bezier curve in this matrix
	Matrix3f diff;
  diff = lLeftBezierContour.block(1, 0, 3, 3) - lLeftBezierContour.block(0, 0, 3, 3);
  lLeftBezierConsts.resize(3);
  lLeftBezierConsts[0] = 3 * diff.row(0) - 6 * diff.row(1) + 3 * diff.row(2);
  lLeftBezierConsts[1] = -6 * diff.row(0) + 6 * diff.row(1);
  lLeftBezierConsts[2] = 3 * diff.row(0);
  lLeftBezierContour = bezierMat * lLeftBezierContour; 
  
	for (int i = 0; i < lRightBezierContour.rows(); ++i) {
		for (int j = 0; j < lRightBezierContour.cols(); ++j) {
			// right side of left foot.
			lRightBezierContour(i, j) = leftFootContour[i][j];
		}
	}
	// right side of left foot defined by one bezier curve in this matrix
	diff = lRightBezierContour.block(1, 0, 3, 3) - lRightBezierContour.block(0, 0, 3, 3);
  lRightBezierConsts.resize(3);
  lRightBezierConsts[0] = 3 * diff.row(0) - 6 * diff.row(1) + 3 * diff.row(2);
  lRightBezierConsts[1] = -6 * diff.row(0) + 6 * diff.row(1);
  lRightBezierConsts[2] = 3 * diff.row(0);
	lRightBezierContour = bezierMat * lRightBezierContour; 
	
	// Right foot consants
	for (int i = 0; i < rLeftBezierContour.rows(); ++i) {
		for (int j = 0; j < rLeftBezierContour.cols(); ++j) {
			// left side of right foot.
			rLeftBezierContour(i, j) = rightFootContour[i][j];
		}
	}
	// left side of left foot defined by one bezier curve in this matrix
  diff = rLeftBezierContour.block(1, 0, 3, 3) - rLeftBezierContour.block(0, 0, 3, 3);
  rLeftBezierConsts.resize(3);
  rLeftBezierConsts[0] = 3 * diff.row(0) - 6 * diff.row(1) + 3 * diff.row(2);
  rLeftBezierConsts[1] = -6 * diff.row(0) + 6 * diff.row(1);
  rLeftBezierConsts[2] = 3 * diff.row(0);
	rLeftBezierContour = bezierMat * rLeftBezierContour;
	
	for (int i = 0; i < rRightBezierContour.rows(); ++i) {
		for (int j = 0; j < rRightBezierContour.cols(); ++j) {
			// right side of right foot.
			rRightBezierContour(i, j) = rightFootContour[i+4][j];
		}
	}
	// right side of right foot defined by one bezier curve in this matrix
  diff = rRightBezierContour.block(1, 0, 3, 3) - rRightBezierContour.block(0, 0, 3, 3);
  rRightBezierConsts.resize(3);
  rRightBezierConsts[0] = 3 * diff.row(0) - 6 * diff.row(1) + 3 * diff.row(2);
  rRightBezierConsts[1] = -6 * diff.row(0) + 6 * diff.row(1);
  rRightBezierConsts[2] = 3 * diff.row(0);
  rRightBezierContour = bezierMat * rRightBezierContour; 
}
*/
bool
KickModule::setKickSupportLegs()
{
  if (ballPosition[1] <= -0.06) {
    if (targetAngle >= 0 && targetAngle <= 15 * M_PI / 180) 
      kickLeg = CHAIN_R_LEG;
  } else if (ballPosition[1] > -0.06 && ballPosition[1] <= -0.03) {
    kickLeg = CHAIN_R_LEG;
  } else if (ballPosition[1] > -0.03 && ballPosition[1] <= -0.01) {
    if (targetAngle >= 0) 
      kickLeg = CHAIN_R_LEG;
  } else if (ballPosition[1] > -0.01 && ballPosition[1] < 0.01) {
    if (targetAngle > 15 * M_PI / 180) 
      kickLeg = CHAIN_R_LEG;
    else if (targetAngle < 15 * M_PI / 180) 
      kickLeg = CHAIN_L_LEG;
  } else if (ballPosition[1] >= 0.01 && ballPosition[1] < 0.03) {
    if (targetAngle <= 0) 
      kickLeg = CHAIN_L_LEG;
  } else if (ballPosition[1] >= 0.03 && ballPosition[1] < 0.06) {
    kickLeg = CHAIN_L_LEG;
  } else if (ballPosition[1] >= 0.06) {
    if (targetAngle <= 0 && targetAngle >= -15 * M_PI / 180) 
      kickLeg = CHAIN_L_LEG;
  }
  supportLeg = kickLeg == CHAIN_R_LEG ? CHAIN_L_LEG : CHAIN_R_LEG;
  if (kickLeg == CHAIN_L_LEG || kickLeg == CHAIN_R_LEG)
    return true;
  return false;
}

bool
KickModule::setTransformFrames() throw (BehaviorException)
{
  try {
    if (kickLeg != supportLeg && 
        (kickLeg == CHAIN_L_LEG || 
        kickLeg == CHAIN_R_LEG) && 
        (supportLeg == CHAIN_L_LEG || 
        supportLeg == CHAIN_R_LEG)) 
    {
      torsoToSupport = kM->getForwardEffector(supportLeg, ANKLE);
      supportToKick = 
        MathsUtils::getTInverse(torsoToSupport) * 
        kM->getForwardEffector(kickLeg, ANKLE);
      return true;
    } else {
      throw BehaviorException(
        this,
        "Cannot set transformations for undefined kick and support legs.",
        false,
        EXC_INVALID_BEHAVIOR_SETUP
      );
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    return false;
  }
 
}
/*
bool KickModule::findBezierAtVecNormal(
  Vector3f& contourPoint,
  const Matrix<float, 4, 3>& contourMat,
  const vector<Vector3f>& contourConsts, 
  const Vector3f& vec)
{
  cout << contourConsts[0] << endl;
  cout << contourConsts[1] << endl;
  cout << contourConsts[2] << endl;
  cout << vec << endl;
	float c1, c2, c3;
	c1 = contourConsts[0].dot(vec);
	c2 = contourConsts[1].dot(vec);
	c3 = contourConsts[2].dot(vec);
	// Solve the quadratic equation
  float t = // Find bezier parameter where vec becomes normal to it
    (-c2 - sqrt(c2 * c2 - 4 * c1 * c3)) / (2 * c1);
  Vector4f tVector;
  tVector << 1, t, pow(t, 2), pow(t, 3);
  // Find the coordinates on bezier where vec becomes normal to it
  if ((0 <= t && t <= 1) && (t > 0.0001)) {
		contourPoint = (tVector.transpose() * contourMat).transpose();
    return true;
  }
  return false;
}
*/
bool
KickModule::setEndEffectorXY(const float& angle)
{
  try {
    if (kickLeg != CHAIN_L_LEG && kickLeg != CHAIN_R_LEG){
      throw BehaviorException(
          this,
          "No kick leg defined to find the end-effector.",
          false,
          EXC_INVALID_BEHAVIOR_SETUP
        );
    }
  } catch (BehaviorException& e) {
    cout << e.what();
    return false;
  }
  
  bool success;
  Vector3f contourPoint;
  auto normal = Vector3f(cos(angle), sin(angle), 0.f);
  if (kickLeg == CHAIN_L_LEG) {
    success = lFootContour->findNormalToVec(normal, contourPoint);
    //cout << "contourPoint: " << contourPoint << endl;
    if (!success)
      return false;
    endEffector(0, 3) = contourPoint[0];
    endEffector(1, 3) = contourPoint[1];
    endEffector(2, 3) = contourPoint[2];
    kM->setEndEffector(kickLeg, KICK_EE, endEffector.block(0, 3, 4, 1));
    return true;
  } else if (kickLeg == CHAIN_R_LEG) {
    success = rFootContour->findNormalToVec(normal, contourPoint);
    //cout << "contourPoint: " << contourPoint << endl;
    if (!success)
      return false;
    endEffector(0, 3) = contourPoint[0];
    endEffector(1, 3) = contourPoint[1];
    endEffector(2, 3) = contourPoint[2];
    kM->setEndEffector(kickLeg, KICK_EE, endEffector.block(0, 3, 4, 1));
    return true;
  }
}

void
KickModule::setEndEffectorZX(const float& t)
{
  /*Matrix4f bezierMat;
  Matrix<float, 4, 3> contourMat;
  bezierMat << 1, 0, 0, 0, -3, 3, 0, 0, 3, -6, 3, 0, -1, 3, -3, 1;
  Vector4f tVector;
  Vector3f contourPoint;
  fstream footCurveLog;
  footCurveLog.open(
	(ConfigManager::getLogsDirPath() + string("KickModule/FootCurveZX.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc);
  footCurveLog << "# t     X     Y    Z" << endl;
  footCurveLog.close();
  contourMat << zxFootContour[0][0], zxFootContour[0][1], zxFootContour[0][2], zxFootContour[1][0], zxFootContour[1][1], zxFootContour[1][2], zxFootContour[2][0], zxFootContour[2][1], zxFootContour[2][2], zxFootContour[3][0], zxFootContour[3][1], zxFootContour[3][2];
  contourMat = bezierMat * contourMat;
  tVector << 1, t, pow(t, 2), pow(t, 3);
  contourPoint = (tVector.transpose() * contourMat).transpose();
  endEffector(0, 3) = contourPoint[0];
  endEffector(2, 3) = contourPoint[2];
  footCurveLog.open(
	(ConfigManager::getConfigDirPath() + string("KickModule/FootCurveZX.txt")).c_str(),
    std::ofstream::out | std::ofstream::trunc);
  for (float ti = 0.0; ti <= 1.0; ti = ti + 0.01) {
    tVector(0, 0) = 1;
    tVector(1, 0) = ti;
    tVector(2, 0) = pow(ti, 2);
    tVector(3, 0) = pow(ti, 3);
    Vector3f point = (tVector.transpose() * contourMat).transpose();
    footCurveLog << ti << "    " << point[0] << "    " << point[1] << "    " << point[2] << endl;
  }
  footCurveLog.close();*/
}

template <typename Scalar>
bool checkFootCollision(
    const Matrix<Scalar, Dynamic, 1>& kickAngles)
{
  unsigned chainStart = this->kM->getLinkChain(kickLeg)->start;
  this->kM->setJointPositions(chainStart, kickAngles, JointStateType::SIM); // impact pose joints
  auto supportToKick = MathsUtils::getTInverse(this->torsoToSupport) * this->kM->getForwardEffector(this->kickLeg, ANKLE, JointStateType::SIM);
  // Get foot configuration
  cout << "Here" << endl;
  Matrix<Scalar, 3, 1> footSize;
  Matrix<Scalar, 2, 1> originShift;
  GET_CONFIG(
    "PathPlanner",
    (Scalar, Foot.sizeX, footSize[0]),
    (Scalar, Foot.sizeY, footSize[1]),
    (Scalar, Foot.sizeZ, footSize[2]),
    (Scalar, Foot.originShiftX, originShift[0]),
    (Scalar, Foot.originShiftY, originShift[1]),
  )
  cout << "Here" << endl;
  // Finding left foot boundary
  Matrix<Scalar, 4, 4> flTop, flBottom;
  flTop(0, 3) = footSize[0] / 2 + originShift[0];
  flTop(1, 3) = footSize[1] / 2 + originShift[1];
  flTop(2, 3) = -footHeight + footSize[2] / 2;
  flBottom(0, 3) = -footSize[0] / 2 + originShift[0];
  flBottom(1, 3) = -footSize[1] / 2 + originShift[1];
  flBottom(2, 3) = -footHeight + footSize[2] / 2;
  Matrix<Scalar, 4, 4> frTop, frBottom;
  frTop(0, 3) = footSize[0] / 2 + originShift[0];
  frTop(1, 3) = footSize[1] / 2 + originShift[1];
  frTop(2, 3) = -footHeight + footSize[2] / 2;
  frBottom(0, 3) = -footSize[0] / 2 + originShift[0];
  frBottom(1, 3) = -footSize[1] / 2 + originShift[1];
  frBottom(2, 3) = -footHeight + footSize[2] / 2;
  cout << "Here" << endl;
  if (this->kickLeg == CHAIN_R_LEG) {
    // Check intersection of line flTop_flBottom with frTop_frBottom
    // fl is used for kicking leg here
    flTop = supportToKick * flTop;
    flBottom = supportToKick * flBottom;
    auto l1 = Vec<Scalar, 4>(flTop(0, 3), flTop(1, 3), flBottom(0, 3), flBottom(1, 3));
    auto l2 = Vec<Scalar, 4>(frTop(0, 3), frTop(1, 3), frBottom(0, 3), frBottom(1, 3));
    auto image = Mat(Size(100, 100), cv::Scalar(0), CV_8UC1);
    cv::line(image, Point(l1[0]*10, l1[1]*10), Point(l1[2]*10, l1[3]*10)), cv::Scalar(255));
    cv::line(image, Point(l2[0]*10, l2[1]*10), Point(l2[2]*10, l2[3]*10)), cv::Scalar(255));
    auto p = Utils::VisionUtils::findIntersection(l1, l2);
    Utils::VisionUtils::drawPoint(Point(p.x*10, p.y*10), image);
    imshow("Footlines", image);
    waitKey(0);
  }
  return true;
}

void
KickModule::setupPosture()
{
  PRINT("KickModule.setupPosture()")
  if (getBehaviorCast()->postureConfig)
    setupChildRequest(getBehaviorCast()->postureConfig);
  else 
    setupChildRequest(
      boost::make_shared<MBPostureConfig>(PostureState::STAND, 1.f)
    );
}
/*
void
KickModule::logFootContours()
{
  bool log1, log2, log3, log4 = false;

  string pathToLogs = ConfigManager::getConfigDirPath() + string("KickModule/");
  log1 = KickUtils::checkFootContourLog(pathToLogs + "FootCurveLeftXY.txt");
  log2 = KickUtils::checkFootContourLog(pathToLogs + "FootCurveRightXY.txt");
  log3 = KickUtils::checkFootContourLog(pathToLogs + "FootCurveLeftZX.txt");
  log4 = KickUtils::checkFootContourLog(pathToLogs + "FootCurveRightZX.txt");
  if (log1) KickUtils::logContour(
    leftFootContour,
    8,
    pathToLogs + "FootCurveLeftXY.txt",
    0.05);
  if (log2) KickUtils::logContour(
    rightFootContour,
    8,
    pathToLogs + "FootCurveRightXY.txt",
    -0.05);
  if (log3) KickUtils::logContour(
    zxFootContour,
    4,
    pathToLogs + "FootCurveLeftZX.txt",
    0.05);
  if (log4) KickUtils::logContour(
    zxFootContour,
    4,
    pathToLogs + "FootCurveRightZX.txt",
    -0.05);
}
*//*
void
KickModule::setupGnuPlotConfig()
{
  gp << "set style arrow 1 head filled size screen 0.01,15,45 ls 1\n";
  //! Setup line style for square symbols
  gp << "set style line 1 lc rgb 'black' pt 5\n";
  //! Setup line style with circular symbols
  gp << "set style line 2 lc rgb 'black' pt 7 ps 0.1\n";
  //! Setup line style with triangular symbols 
  gp << "set style line 3 lc rgb 'black' pt 9\n";
  //! Set plot to equal width/height ratio
  gp << "set view equal xyz\n";
  gp << "set size ratio -1\n";
  //! Set x and y ranges of plot
  gp << "set xrange [-1:1];set yrange [-1:1];set zrange [0:1]\n";
  //! Setup an empty plot
  gp << "plot 1/0 t''\n";
}
*//*
void
KickModule::plotFootSurfaces()
{
  string logsPath = ConfigManager::getLogsDirPath() + string("KickModule/");
  string lLogPath = logsPath + string("FootCurveLeftXY.txt");
  string rLogPath = logsPath + string("FootCurveRightXY.txt");
  string zxLLogPath = logsPath + string("FootCurveLeftZX.txt");
  string zxRLogPath = logsPath + string("FootCurveRightZX.txt");
  fstream lLog, rLog, zxLLog, zxRLog, surfacePoints;
  lLog.open(lLogPath, std::fstream::in);
  rLog.open(rLogPath, std::fstream::in);
  zxLLog.open(zxLLogPath, std::fstream::in);
  zxRLog.open(zxRLogPath, std::fstream::in);
  if (!rLog || !lLog || !zxLLog || !zxRLog) throw("Foot contour log files not found.\n");

  Mat lFoot, rFoot;
  makeFootSurfaces3D(true, lLog, zxLLog, lFoot);
  makeFootSurfaces3D(false, rLog, zxRLog, rFoot);

  surfacePoints.open(
    logsPath + "surfacePointsLeft.txt",
    fstream::out | fstream::trunc);
  surfacePoints << "# X Y Z" << endl;
  for (size_t r = 0; r < lFoot.rows; ++r) {
    for (size_t c = 0; c < lFoot.cols; ++c) {
      surfacePoints << " " << lFoot.at < Vec3f > (r, c)[0] << " " << lFoot.at < Vec3f > (r, c)[1] << " " << lFoot.at < Vec3f > (r, c)[2] << endl;
    }
    surfacePoints << endl;
  }
  surfacePoints.close();

  surfacePoints.open(
    logsPath + "surfacePointsRight.txt",
    fstream::out | fstream::trunc);
  surfacePoints << "# X Y Z" << endl;

  for (size_t r = 0; r < rFoot.rows; ++r) {
    for (size_t c = 0; c < rFoot.cols; ++c) {
      surfacePoints << " " << rFoot.at < Vec3f > (r, c)[0] << " " << rFoot.at < Vec3f > (r, c)[1] << " " << rFoot.at < Vec3f > (r, c)[2] << endl;
    }
    surfacePoints << endl;
  }
  surfacePoints.close();
  gp << "set xlabel 'y-Axis'\n";
  gp << "set ylabel 'x-Axis'\n";
  gp << "set zlabel 'z-Axis'\n";
  Vector3f framePos(0.f, 0.f, leftFootContour[0][2]);
  Vector3f frameRot(0.f, 0.f, M_PI / 2);
  KickUtils::drawFrame3D(gp, framePos, frameRot); // taking height
  gp << "splot '" << logsPath + "surfacePointsLeft.txt" << "' using 1:2:3 with lines lw 1 lc 3, '" << logsPath + "surfacePointsRight.txt" << "' using 1:2:3 with lines lw 1 lc 3\n";

}
*//*
void
KickModule::makeFootSurfaces3D(const bool& leftFoot, fstream& log,
  fstream& zxLog, Mat& surfaceMat)
{
  unsigned rows = KickUtils::countLines(log) - 1;
  unsigned cols = KickUtils::countLines(zxLog) - 1;
  surfaceMat = Mat(Size(cols, rows), CV_32FC3);
  string line;
  unsigned r = 0;
  while (getline(log, line)) {
    if (line.find('#') != line.npos) continue;
    vector < string > parts;
    split(parts, line, boost::is_any_of(" "));
    if (parts.size() >= 7) {
      float transX, transY;
      DataUtils::stringToVar(parts[1], transX);
      DataUtils::stringToVar(parts[2], transY);
      float tangentX, tangentY;
      DataUtils::stringToVar(parts[4], tangentX);
      DataUtils::stringToVar(parts[5], tangentY);
      float angle = atan2(-tangentX, tangentY);
      if (!leftFoot) angle += M_PI;
      Matrix4f trans = KickUtils::rotZ(angle);
      trans(0, 3) = transX;
      trans(1, 3) = transY;
      string lineZX;
      unsigned ln2 = 0;
      float startX, startY;
      unsigned c = 0;
      while (getline(zxLog, lineZX)) {
        ++ln2;
        if (lineZX.find('#') != lineZX.npos) continue;
        vector < string > parts;
        split(parts, lineZX, boost::is_any_of(" "));
        if (parts.size() >= 4) {
          Vector4f zxVector;
          DataUtils::stringToVar(parts[1], zxVector[0]);
          DataUtils::stringToVar(parts[2], zxVector[1]);
          DataUtils::stringToVar(parts[3], zxVector[2]);
          if (ln2 == 2) {
            startX = zxVector[0];
            startY = zxVector[1];
          }
          zxVector[0] -= startX;
          zxVector[1] -= startY;
          zxVector[3] = 1;
          zxVector = trans * zxVector;
          surfaceMat.at < Vec3f > (r, c)[0] = -zxVector[1];
          surfaceMat.at < Vec3f > (r, c)[1] = zxVector[0];
          surfaceMat.at < Vec3f > (r, c)[2] = zxVector[2];
          c++;
        }
      }
      r++;
      zxLog.clear();
      zxLog.seekg(0, ios::beg);
    }
  }
  log.close();
  zxLog.close();
}
*/
/*
void
KickModule::initiate()
{
  auto& type = getBehaviorCast()->type;
  PRINT("KickModule type: " << (unsigned)type)
  if (type == MBKickTypes::VM_BASED) {
    if (!setupVMBasedKick()) return;
  } else if (type == MBKickTypes::FIXED_VELOCITY) {
    if (!setupFixedVelocityKick()) return;
  }
  setupBalanceModule();
  setupPostureModule();
  setupBallTracking();
  inBehavior = true;
}

void
KickModule::finishBehaviorSafely()
{
  inBehavior = false;
}

void
KickModule::update()
{
  //! If behavior finished
  if (!inBehavior) return;

  //! If a child behavior is running
  if (!reqChildBehaviorState()) return;

  //PRINT("KickModule.update()")
  //! If behavior kill is called
  if (behaviorConfig->killBehavior) {
    if (getBehaviorCast()->ballTrack) {
      htModule->finishBehaviorSafely();
    }
    finishBehaviorSafely();
    return;
  }

  //! If ball tracking is on
  if (getBehaviorCast()->ballTrack) {
    //PRINT("KickModule.htModuleUpdate()")
    htModule->update();
  }

  if (behaviorState == preKickPostureInit) {
    //PRINT("KickModule.preKickPostureInit()")
    if (OVAR(PostureState, MotionModule::postureState) != PostureState::STAND) {
      postureModule->initiate();
      behaviorState = preKickPostureExec;
    } else {
      behaviorState = preKickBalanceInit;
    }
  } else if (behaviorState == preKickPostureExec) {
    //PRINT("KickModule.preKickPostureExec()")
    if (postureModule->getState()) postureModule->update();
    else behaviorState = preKickBalanceInit;
  } else if (behaviorState == preKickBalanceInit) {
    //PRINT("KickModule.preKickBalanceInit()")
    balanceModule->initiate();
    behaviorState = preKickBalanceExec;
  } else if (behaviorState == preKickBalanceExec) {
    //PRINT("KickModule.preKickBalanceExec()")
    if (balanceModule->getState()) {
      balanceModule->update();
    } else {
      behaviorState = kickSetup;
    }
  } else if (behaviorState == kickSetup) {
    //PRINT("KickModule.kickSetup()")
    setForwardTransform();
    correctInitParams();
    setEndEffector();
    //setBallHitPose();
    //setRetractionPose();
    defineTrajectory();
    //plotKick();
    requestExecution();
    behaviorState = kickExecution;
  } else if (behaviorState == kickExecution) {
    //PRINT("KickModule.kickExecution()")
    if (timeStep > timeToKick + cycleTime / 2) behaviorState = postKickInit;
    else timeStep = timeStep + cycleTime;
  } else if (behaviorState == postKickInit) {
    //PRINT("KickModule.postKickInit()")
    auto pConfig = boost::make_shared<MBPostureConfig>();
    pConfig->timeToReachP = 1.f;
    pConfig->posture = PostureState::STAND;
    postureModule->setBehaviorConfig(pConfig);
    postureModule->initiate();
    behaviorState = postKickExec;
  } else if (behaviorState == postKickExec) {
    //PRINT("KickModule.postKickExec()")
    if (postureModule->getState()) postureModule->update();
    else inBehavior = false;
  }
}*/
/*
bool
KickModule::setupFixedVelocityKick()
{
  auto& ball = getBehaviorCast()->ball;
  auto& reqVel = getBehaviorCast()->reqVel;
  if (ball.x != -1.f && reqVel.x != -1.f) {
    ballPosition = Vector3f(ball.x, ball.y, -footHeight + ballRadius);
    desImpactVel = Vector3f(reqVel.x, reqVel.y, 0.f);
    auto velMag = norm(reqVel);
    ballToTargetUnit = Vector3f(reqVel.x / velMag, reqVel.y / velMag, 0.f);
    targetAngle = atan2(reqVel.y, reqVel.x);
    endEffector.setIdentity();
    retractionPose.setIdentity();
    impactPose.setIdentity();
    supportToKick.setIdentity();
    setKickLeg();
    supportLeg = kickLeg == CHAIN_R_LEG ? CHAIN_L_LEG : CHAIN_R_LEG;
    return true;
  } else {
    PRINT("Required kick parameters 'ball' & 'reqVel' not defined")
    return false;
  }
}

bool
KickModule::setupVMBasedKick()
{
  auto& ball = getBehaviorCast()->ball;
  auto& targetInField = getBehaviorCast()->target;
  if (ball.x != -1.f && targetInField.x != -1.f) {
    auto& rState = IVAR(RobotPose2D<float>, MotionModule::robotPose2D);
    Matrix3f rotation;
    MathsUtils::makeRotationZ(rotation, rState.theta);
    Matrix4f robotFrameT = MathsUtils::makeTransformation(
      rotation,
      rState.x,
      rState.y,
      0.f);
    Matrix4f robotFrameTInv = MathsUtils::getTInverse(robotFrameT);
    Vector4f temp = robotFrameTInv * Vector4f(
      targetInField.x,
      targetInField.y,
      0.f,
      1.f);
    float height = -footHeight + ballRadius;
    ballPosition = Vector3f(ball.x, ball.y, height);
    targetPosition = Vector3f(temp[0], temp[1], height);
    OVAR(Point2f, MotionModule::kickTarget) = targetInField;
    ballToGoal = targetPosition - ballPosition;
    ballToTargetUnit = ballToGoal / ballToGoal.norm();
    targetDistance = ballToGoal.norm();
    targetAngle = atan2(ballToTargetUnit[1], ballToTargetUnit[0]);
    endEffector.setIdentity();
    retractionPose.setIdentity();
    impactPose.setIdentity();
    supportToKick.setIdentity();
    setKickLeg();
    if (kickLeg == -1) return false;
    supportLeg = kickLeg == CHAIN_R_LEG ? CHAIN_L_LEG : CHAIN_R_LEG;
    return true;
  } else {
    PRINT("Required kick parameters 'ball' & 'target' not defined")
    return false;
  }
}*/
/*
void
KickModule::setupBallTracking()
{
  PRINT("KickModule.setupBallTracking()")
  if (getBehaviorCast()->ballTrack) {
    htModule = boost::shared_ptr < HeadTracking > (new HeadTracking(
      motionModule));
    auto htConfig =
      boost::make_shared < MBHeadTrackingConfig > (MBHeadTrackingTypes::FOLLOW_BALL);
    htModule->setBehaviorConfig(htConfig);
    htModule->initiate();
  }
}



void
KickModule::requestExecution()
{
  if (getBehaviorCast()->type == MBKickTypes::VM_BASED) {
    unsigned chainStart = kM->getChainStart(kickLeg);
    unsigned chainSize = kM->getChainSize(kickLeg);
    vector<unsigned> jointIds;
    for (size_t i = chainStart + 1; i < chainStart + chainSize; ++i)
      jointIds.push_back(i);
    AL::ALValue jointTimes;
    AL::ALValue jointPositions;
    jointTimes.clear();
    jointPositions.clear();
    jointTimes.arraySetSize(chainSize - 1);
    jointPositions.arraySetSize(chainSize - 1);
    //if(kickLeg == CHAIN_L_LEG) {
    // jointPositions[0].arrayPush( 9 * M_PI / 180);
     //jointPositions[1].arrayPush(-40 * M_PI / 180);
     //jointPositions[2].arrayPush(80 * M_PI / 180);
//     jointPositions[3].arrayPush(-40 * M_PI / 180);
     //jointPositions[4].arrayPush(-18 * M_PI / 180);
     //} else {
     //jointPositions[0].arrayPush( -9 * M_PI / 180);
     //jointPositions[1].arrayPush(-40 * M_PI / 180);
     //jointPositions[2].arrayPush(80 * M_PI / 180);
     //jointPositions[3].arrayPush(-40 * M_PI / 180);
     //jointPositions[4].arrayPush(18 * M_PI / 180);
     //}
     //for(size_t i = 0; i < chainSize-1; ++i) {
     //jointTimes[i].arrayPush(balanceTime);
     //}
    unsigned trajStep = 0;
    while (true) {
      for (int i = 0; i < chainSize - 1; ++i) {
        jointPositions[i].arrayPush(jointTrajectories[i][trajStep]);
        jointTimes[i].arrayPush(balanceTime + (trajStep + 1) * 0.01);
      }
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }
    timeToKick = (trajStep + 1) * 0.01;
    //cout << "jointPositions : " << jointTimes<< endl;
    //cout << "totaltime : " << timeToKick << endl;
    timeStep = 0.f;
    naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
    //plotJointTrajectories();
  } else {
    unsigned chainStart = kM->getChainStart(kickLeg);
    unsigned chainSize = kM->getChainSize(kickLeg);
    vector<unsigned> jointIds;
    for (size_t i = chainStart; i < chainStart + chainSize; ++i)
      jointIds.push_back(i);
    AL::ALValue jointTimes;
    AL::ALValue jointPositions;
    jointTimes.clear();
    jointPositions.clear();
    jointTimes.arraySetSize(chainSize);
    jointPositions.arraySetSize(chainSize);
    unsigned trajStep = 0;
    while (true) {
      for (int i = 0; i < chainSize; ++i) {
        jointPositions[i].arrayPush(jointTrajectories[i][trajStep]);
        jointTimes[i].arrayPush((trajStep + 1) * cycleTime);
      }
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }

    //cout << "jointPositions: " << jointPositions << endl;
    //cout << "jointTimes: " << jointTimes << endl;
    timeToKick = (trajStep + 1) * cycleTime;
    //cout << "totaltime : " << timeToKick << endl;
    timeStep = 0.f;
    naoqiJointInterpolation(jointIds, jointTimes, jointPositions, false);
    //plotJointTrajectories();
  }
}

void
KickModule::correctInitParams()
{
  auto& type = getBehaviorCast()->type;
  if (type == MBKickTypes::VM_BASED) {
    footSpacing = supportToKick(1, 3) / 2;
    ballPosition[1] = ballPosition[1] + footSpacing;
    targetPosition[1] = targetPosition[1] + footSpacing;
  } else if (type == MBKickTypes::FIXED_VELOCITY) {
    footSpacing = supportToKick(1, 3) / 2;
    ballPosition[1] = ballPosition[1] + footSpacing;
  }
}

void
KickModule::setRetractionPose()
{
  retractionPose.setIdentity();
  float angle = targetAngle;
  if (targetAngle >= 25 * M_PI / 180) angle = 25 * M_PI / 180;
  else if (targetAngle <= -25 * M_PI / 180) angle = -25 * M_PI / 180;

  if (getBehaviorCast()->type == MBKickTypes::VM_BASED) {
    float relDistX = 0.03;
    float relDistY = 0.03;
    float relHeight = 0.025;
    retractionPose(0, 3) = endEffector(0, 3) / 2 - relDistX * cos(angle);
    retractionPose(1, 3) = -relDistY * sin(angle);
    retractionPose(2, 3) = endEffector(2, 3) / 2 + relHeight;
    retractionPose = supportToKick * retractionPose;
    cout << "retractionPose Start: " << retractionPose << endl;
  } else {
    if (targetDistance > 2.5) {
      retractionPose(0, 3) = ballPosition[0] - ballRadius - 0.05 * cos(angle);
      retractionPose(1, 3) = ballPosition[1] - 0.05 * sin(angle);
      retractionPose(2, 3) = 0.01;
    } else {
      retractionPose(0, 3) = ballPosition[0] - ballRadius - 0.03 * cos(angle);
      retractionPose(1, 3) = ballPosition[1] - 0.03 * sin(angle);
      retractionPose(2, 3) = 0.01;
    }
  }
  //cout << "retractionPose: " << retractionPose << endl;
  ////retractionPose = (ballHitPoint + endEffector)/2 - (ballHitPoint - endEffector).norm()/2*ballToTargetUnit;
  //retractionPose[2] = ballHitPoint[2] + 0.025;
  //retractionPose = endEffector + retractionPose;
  cout << "ForwardTransform: " << supportToKick << endl;
  //retractionPose = supportToKick*retractionPose;
}

void
KickModule::setBallHitPose()
{
  impactPose(0, 3) = ballPosition[0] - (ballRadius) * ballToTargetUnit[0];
  impactPose(1, 3) = ballPosition[1] - (ballRadius) * ballToTargetUnit[1];
  impactPose(2, 3) = ballPosition[2];
}

void
KickModule::defineTrajectory()
{
  //trajectoryPlanner->setStepSize(0.01f);
  // trajectoryPlanner->setTrajectoryTime(1.0f);
   //trajectoryPlanner->setTrajectoryKnots();
   //vector<Vector3f> trajectoryRefPoints;
   //trajectoryRefPoints.push_back(Maths::firstThree(endEffector));
   //trajectoryRefPoints.push_back(Maths::firstThree(retractionPoint));
   //trajectoryRefPoints.push_back(Maths::firstThree(ballHitPoint));
   //trajectoryPlanner->setReferencePoints(trajectoryRefPoints);
   //Vector3f initialVelocity = Vector3f(-0.0, -0.0, 0.0);//Vector3f::Zero();
   //Vector3f finalVelocity = Vector3f(0.5*ballToTargetUnit[0], 0.5*ballToTargetUnit[1], 0.0); //0.5, 0.5, 0.0 in 45 degrees
   //trajectoryPlanner->defineViaPoints(initialVelocity, finalVelocity);
  if (getBehaviorCast()->type == MBKickTypes::FIXED_VELOCITY) {
    cout << "supportToKick:\n" << supportToKick << endl;
    torsoToSLeg = kM->getForwardEffector(
      supportLeg,
      ANKLE);
    auto eeTrans = supportToKick * endEffector;
    //! Define via points for final trajectory.
    const float balldx = ballRadius * ballToTargetUnit[0];
    const float balldy = ballRadius * ballToTargetUnit[1];
    Matrix4f postImpactPose2, postImpactPose1;
    postImpactPose2 = eeTrans;
    postImpactPose1.setIdentity();
    postImpactPose1(0, 3) = ballPosition[0];
    postImpactPose1(1, 3) = ballPosition[1];
    postImpactPose1(2, 3) = ballPosition[2];
    //! Define via points for constant velocity phase.
    impactPose.setIdentity();
    impactPose(0, 3) = ballPosition[0] - balldx;
    impactPose(1, 3) = ballPosition[1] - balldy;
    impactPose(2, 3) = ballPosition[2];
    //! Define via points for preImpactPose phase.
    Matrix4f preImpactPose2, preImpactPose1;
    preImpactPose2.setIdentity();
    preImpactPose2(0, 3) = (impactPose(0, 3) + eeTrans(0, 3)) / 2;
    preImpactPose2(1, 3) = (impactPose(1, 3) + eeTrans(1, 3)) / 2;
    preImpactPose2(2, 3) = impactPose(2, 3);
    preImpactPose2(0, 3) =
      preImpactPose2(0, 3) - ballToTargetUnit[0] * ballRadius;
    preImpactPose2(1, 3) =
      preImpactPose2(1, 3) - ballToTargetUnit[1] * ballRadius;
    preImpactPose1 = eeTrans;
    cout << "EndEffector: " << endEffector << endl;
    cout << "ImpactPose: " << impactPose << endl;
    //findBestEEAndImpactPose();
    cout << "ImpactPose: " << impactPose << endl;
    unsigned chainSize = kM->getChainSize(kickLeg);
    unsigned chainStart = kM->getChainStart(kickLeg);
    cPoses.clear();
    vector<Matrix4f> cPosesT;
    cPoses.push_back(preImpactPose1);
    cPoses.push_back(preImpactPose2);
    cPoses.push_back(impactPose);
    cPoses.push_back(postImpactPose1);
    cPoses.push_back(postImpactPose2);
    for (int i = 0; i < cPoses.size(); ++i) {
      cPosesT.push_back(torsoToSLeg * cPoses[i]);
    }

    MatrixXf jointPos;
    jointPos.resize(cPosesT.size(), chainSize);
    jointPos.setZero();
    for (int i = 0; i < cPosesT.size(); ++i) {
      vector < VectorXf > angles;
      if (kickLeg == CHAIN_L_LEG) angles = kM->inverseLeftLeg(
        endEffector,
        cPosesT[i]);
      else angles = kM->inverseRightLeg(endEffector, cPosesT[i]);
      if (angles.size() != 0) {
        jointPos.row(i) = angles[0].transpose();
      } else {
        ERROR(
          "The required cartesian pose " << i << " is out of the configuration space of given chain.")
        return;
      }
    }

    //! Setting up pre-impact trajectory.
    //! Required cartesian velocities at initial and final poses
    vector<VectorXf> cBoundVels(2);
    for (int i = 0; i < cBoundVels.size(); ++i) {
      cBoundVels[i].resize(6); //x-y-z, r-p-y
      cBoundVels[i].setZero();
    }
    cBoundVels[1].block(0, 0, 3, 1) = desImpactVel; // x-y-z velocity
    //! Cartesian velocities in torso frame.
    for (int i = 0; i < cBoundVels.size(); ++i)
      cBoundVels[i].block(0, 0, 3, 1) =
        torsoToSLeg.block(0, 0, 3, 3) * cBoundVels[i].segment(0, 3);
    //! Find last impact velocity in joints space using hitPose.
    VectorXf impactJVel;
    kM->setJointPositions(chainStart, jointPos.row(2), KinematicsModule::SIM); // impact pose joints
    MatrixXf jacobian = kM->computeLimbJ(
      kickLeg,
      endEffector,
      KinematicsModule::SIM).block(0, 0, 3, 6);
    impactJVel = MathsUtils::pseudoInverseSolve(
      jacobian,
      VectorXf(cBoundVels[1].block(0, 0, 3, 1)));

    Vector3f direction = torsoToSLeg.block(0, 0, 3, 3) * ballToTargetUnit;
    kM->setChainPositions(kickLeg, jointPos.row(2), KinematicsModule::SIM); // impact pose joints
    float vm;
    kM->computeVirtualMass(
      kickLeg,
      direction,
      endEffector,
      vm,
      KinematicsModule::SIM);
    cout << "Virtual mass in given direction and found pose: " << vm << endl;

    MatrixXf jointBoundVels;
    jointBoundVels.resize(2, chainSize);
    jointBoundVels.setZero();
    jointBoundVels.row(1) = impactJVel.transpose(); // Second row
    VectorXf maxVels;
    maxVels = kM->getChainVelLimits(kickLeg);
    for (int i = 0; i < maxVels.size(); ++i)
      maxVels[i] -= 0.015;

    PRINT("Performing first trajectory optimization...")
    //! Second trajectory optimization
    VectorXf knots;
    knots.resize(2); // 3 poses for pre-impact/post-impact trajectory
    for (int i = 0; i < knots.size(); ++i)
      knots[i] = 0.2;
    auto cb1 = CubicSpline(
      cycleTime,
      chainSize,
      0,
      jointPos.block(0, 0, 3, chainSize),
      knots,
      jointBoundVels);
    auto cbopt = CbOptimizer(motionModule, kickLeg, supportLeg, cb1);
    cbopt.setZmpCons(true);
    cbopt.optimizeKnots();
    vector<float> trajTime;
    cb1.getTrajectories(jointTrajectories, trajTime, 0);

    //! Second trajectory optimization
    PRINT("Performing second trajectory optimization...")
    vector < vector<float> > jointTrajectories2;
    jointBoundVels.setZero();
    jointBoundVels.row(0) = impactJVel.transpose(); // First row
    knots.resize(2); // 3 poses for pre-impact/post-impact trajectory
    for (int i = 0; i < knots.size(); ++i)
      knots[i] = 0.2;
    auto cb2 = CubicSpline(
      cycleTime,
      chainSize,
      0,
      jointPos.block(2, 0, 3, chainSize),
      knots,
      jointBoundVels);
    //cb2.optimizeKnots(maxVels);
    cb2.getTrajectories(jointTrajectories2, trajTime, 0);

    for (int i = 0; i < jointTrajectories.size(); ++i) {
      jointTrajectories[i].insert(
        jointTrajectories[i].end(),
        jointTrajectories2[i].begin(),
        jointTrajectories2[i].end());
    }

    ////! Setting up constant velocity impact trajectory.
    ////! Linear change in position with avg velocity equal to impact vel
    //VectorXf jointTimes;
     //jointTimes.resize(chainSize);
    // cout << "test" << endl;
    // cout << jointPos.block(2, 0, 2, chainSize) << endl;
    // cout << "impactJVel: " << impactJVel << endl;
     //cout << (jointPos.block(3, 0, 1, chainSize) - jointPos.block(2, 0, 1, chainSize)).cwiseQuotient(impactJVel) << endl;
    // for (float t = 0; t <= 0.05; t += 0.01) {
    // cout << "t: " << t << endl;
    // auto ji = jointPos.block(2, 0, 1, chainSize).transpose()  ;
    // auto vel = (impactJVel.array() * t).matrix();
    // auto jf = ji + vel;
    // //cout << "ji: " << ji << endl;
     //cout << "vel: " << vel << endl;
   ////  cout << "jf: " << jf.transpose().array() * 180 / M_PI << endl;
    // cout << "hitPoseJionts: " << hitPoseJoints.array() * 180 / M_PI << endl;
    // }

  } else if (getBehaviorCast()->type == MBKickTypes::VM_BASED) {
    virtualMassKick();
  } else {
    vector<VectorXf> cBoundVels(2);
    IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    torsoToSLeg = kM->getForwardEffector(
      supportLeg,
      ANKLE);
    cout << "Forward: " << MathsUtils::getTInverse(torsoToSLeg).format(
      OctaveFmt) << endl;
    cPoses.push_back(supportToKick * endEffector);
    cPoses.push_back(retractionPose);
    //Matrix4f inter;
    // inter.setIdentity();
    //// inter(0, 3) = (retractionPose(0, 3) + impactPose(0, 3)) / 2;
    // inter(1, 3) = (retractionPose(1, 3) + impactPose(1, 3)) / 2;
    // inter(2, 3) = (retractionPose(2, 3) + impactPose(2, 3)) / 2;
    // cPoses.push_back(inter);
    //cPoses.push_back(impactPose);
    //Matrix4f tX, tY;
     //Matrix4f newPose = impactPose;
    // //MathsUtils::makeRotationX(tX, 5*M_PI/180);
     //MathsUtils::makeRotationY(tY, 10*M_PI/180);
    // newPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
    // newPose = torsoToSLeg * newPose;
    // vector<VectorXf> angles = kM->inverseRightLeg(endEffector, newPose);
    // if (angles.size() != 0) {
    // cout << "angles1: " << angles[0][0] * 180 / M_PI << endl;
    // cout << "angles2: " << angles[0][1] * 180 / M_PI << endl;
    // cout << "angles3: " << angles[0][2] * 180 / M_PI << endl;
    // cout << "angles4: " << angles[0][3] * 180 / M_PI << endl;
    // cout << "angles5: " << angles[0][4] * 180 / M_PI << endl;
    // cout << "angles6: " << angles[0][5] * 180 / M_PI << endl; 
    // }
     
    //findBestEEAndImpactPose();
    Matrix4f t;
    MathsUtils::makeRotationY(t, 0 * M_PI / 180);
    cPoses[0].block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
    MathsUtils::makeRotationY(t, 0 * M_PI / 180);
    cPoses[1].block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
    MathsUtils::makeRotationY(t, 0 * M_PI / 180);
    cPoses[2].block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
    vector < vector<float> > jointTrajectories2;
    vector<Matrix4f> cPoses2;
    cPoses2.push_back(cPoses.back());
    Matrix4f interPose = cPoses.back();
    interPose(0, 3) = cPoses.back()(0, 3) + ballRadius / 2;
    interPose(2, 3) = cPoses.back()(2, 3) + ballRadius / 2;
    MathsUtils::makeRotationY(t, 0 * M_PI / 180);
    interPose.block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
    cPoses2.push_back(interPose);
    interPose = cPoses[0];
    interPose(0, 3) = cPoses[0](0, 3) + 0.025;
    interPose(2, 3) = cPoses[0](2, 3) + 0.025;
    MathsUtils::makeRotationY(t, 0 * M_PI / 180);
    interPose.block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
    cPoses2.push_back(interPose);
    cPoses2.push_back(cPoses[0]);
    cout << "endEffector1: " << endl << cPoses[0].format(OctaveFmt) << endl;
    cout << "retractionPose1: " << endl << cPoses[1].format(OctaveFmt) << endl;
    cout << "impactPose1: " << endl << cPoses[2].format(OctaveFmt) << endl;
    for (int i = 0; i < cPoses.size(); ++i) {
      cPoses[i] = torsoToSLeg * cPoses[i];
    }
    for (int i = 0; i < cPoses2.size(); ++i) {
      cPoses2[i] = torsoToSLeg * cPoses2[i];
    }
    cout << "endEffector2: " << endl << cPoses[0].format(OctaveFmt) << endl;
    cout << "retractionPose2: " << endl << cPoses[1].format(OctaveFmt) << endl;
    cout << "impactPose2: " << endl << cPoses[2].format(OctaveFmt) << endl;
    cout << "cPoses2-1: " << endl << cPoses2[0].format(OctaveFmt) << endl;
    cout << "cPoses2-2: " << endl << cPoses2[1].format(OctaveFmt) << endl;
    //cout << "cPoses2-3: " << endl << cPoses2[2].format(OctaveFmt) << endl;
    //cout << "cPoses2-4: " << endl << cPoses2[3].format(OctaveFmt) << endl;
    VectorXf iVel, fVel;
    float vel = 1.0;
    iVel.resize(6);
    fVel.resize(6);
    iVel.setZero();
    fVel.setZero();
    fVel[0] = vel * ballToTargetUnit[0]; // * cos(-10*M_PI/180);
    fVel[1] = vel * ballToTargetUnit[1];
    fVel[2] = 0.0; //vel * sin(-10*M_PI/180);
    iVel.block(0, 0, 3, 1) = torsoToSLeg.block(0, 0, 3, 3) * iVel.segment(0, 3);
    fVel.block(0, 0, 3, 1) = torsoToSLeg.block(0, 0, 3, 3) * fVel.segment(0, 3);
    cBoundVels.push_back(iVel);
    cBoundVels.push_back(fVel);
    //cout << "endEffector" << endEffector << endl;
    cout << "Planning: " << endl;
   //     trajectoryPlanner->cartesianPlanner(
   //  jointTrajectories,
   //  kickLeg,
   //  endEffector,
   //  cPoses,
   //  cBoundVels,
   //  false,
   //  false);
   //  cBoundVels.clear();
    // cBoundVels.push_back(fVel);
    // cBoundVels.push_back(iVel);
   //  trajectoryPlanner->cartesianPlanner(
    // jointTrajectories2,
   //  kickLeg,
   //  endEffector,
   //  cPoses2,
   //  cBoundVels,
  //   true,
   //  false);
    //for (int i = 0; i < jointTrajectories.size(); ++i) {
    // jointTrajectories[i].insert(
    // jointTrajectories[i].end(),
    // jointTrajectories2[i].begin(),
    // jointTrajectories2[i].end()
    // );
     ////cout << "joint[" << i << "]" << endl;
     //for (int j = 0; j < jointTrajectories[i].size(); ++j) {
     //  cout << j << ": " << jointTrajectories[i][j] * 180 / M_PI << endl;
     // }
   //  }
    //trajStep = 0;
  }
}


double
KickModule::minTimeObj(const vector<double>& vars, vector<double>& grad,
  void *data)
{
  //cout << "Finding f.. " << endl;
  double f = 0;
  // Set end-effector position based on bezier parameter t.
  setEndEffectorZX(vars[2]);
  Matrix4f tX, tY;
  Matrix4f newPose = impactPose;
  MathsUtils::makeRotationX(tX, vars[0]);
  MathsUtils::makeRotationY(tY, vars[1]);
  newPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  newPose = torsoToSLeg * newPose;
  vector < VectorXf > angles;
  if (kickLeg == CHAIN_L_LEG) angles = kM->inverseLeftLeg(endEffector, newPose);
  else angles = kM->inverseRightLeg(endEffector, newPose);
  float vm;
  Vector3f direction = torsoToSLeg.block(0, 0, 3, 3) * ballToTargetUnit;
  kM->setChainPositions(kickLeg, angles[0], KinematicsModule::SIM);
  kM->computeVirtualMass(
    kickLeg,
    direction,
    endEffector,
    vm,
    KinematicsModule::SIM);
  float vel = vars[3];
  f = 1 / (vel * vm) + 1 / endEffector(0, 3);
  //cout << "f: " << f << endl;
  return f;
}

double
KickModule::objWrapper(const vector<double> &vars, vector<double> &grad,
  void *data)
{
  KickModule *obj = static_cast<KickModule *>(data);
  return obj->minTimeObj(vars, grad, data);
}

void
KickModule::ineqWrapper(unsigned nCons, double *result, unsigned nVars,
  const double* vars, double* grad, void* data)
{
  KickModule *obj = static_cast<KickModule *>(data);
  return obj->ineqConstraints(nCons, result, nVars, vars, grad, data);
}

void
KickModule::ineqConstraints(unsigned nCons, double *result, unsigned nOptVars,
  const double* vars, double* grad, void* data)
{
  //cout << "Solving Inequality Constraints..." << endl;
  VectorXf fVel;
  float vel = vars[3];
  fVel.resize(3);
  fVel.setZero();
  fVel[0] = vel * ballToTargetUnit[0];
  fVel[1] = vel * ballToTargetUnit[1];
  fVel[2] = 0.0;
  fVel.block(0, 0, 3, 1) = torsoToSLeg.block(0, 0, 3, 3) * fVel.segment(0, 3);
  MatrixXf jacobian = kM->computeLimbJ(
    kickLeg,
    endEffector,
    KinematicsModule::SIM).block(0, 0, 3, 6);
  VectorXf jVels = MathsUtils::pseudoInverseSolve(jacobian, fVel);
  VectorXf velCons = jVels.cwiseAbs() - kM->getChainVelLimits(kickLeg);
  //cout << jVels << endl;
  //cout << kM->getChainVelLimits(kickLeg) << endl;
  for (int i = 0; i < nCons; ++i) {
    result[i] = velCons[i];
  }
  //cout << "Finished solving Inequality Constraints..." << endl;
}

void
KickModule::findBestEEAndImpactPose()
{
  //!Objective function to minimize is virtualMass x velocity;
  //!Hessian for this objective function is unknown.
  //!Gradient for this function is unknown.
  //!4 variables; 2xEuler Angles, end-effector contour parameter t, and velocity.
  nlopt::opt opt(nlopt::LN_COBYLA, 4);
  vector<double> lb(4), ub(4), var0, constraintTols;
  //These are not euler angles rather they are fixed angle rotations.
  lb[0] = -5.0 * M_PI / 180.0; //! Lower bound for x-angle. 
  lb[1] = -10.0 * M_PI / 180.0; //! Lower bound for y-angle. 
  lb[2] = 0.0; //! Lower bound for parameterized curve [0...1].
  lb[3] = 0.01; //! Lower bound for velocity in given direction.
  ub[0] = 5.0 * M_PI / 180.0; //! Upper bound for x-angle. 
  ub[1] = 10.0 * M_PI / 180.0; //! Upper bound for y-angle. 
  ub[2] = 1.0; //! Upper bound for parameterized curve [0...1].
  ub[3] = 2.0; //! Upper bound for velocity in given direction.
  for (int i = 0; i < lb.size(); ++i)
    var0.push_back(lb[i]);

  //! Joint velocity contraint for leg joints;
  unsigned nCons = kM->getChainSize(kickLeg);
  for (int i = 0; i < nCons; ++i) {
    constraintTols.push_back(1e-8);
  }

  opt.add_inequality_mconstraint(KickModule::ineqWrapper, this, constraintTols);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(KickModule::objWrapper, this);
  opt.set_xtol_rel(1e-4);
  opt.set_maxeval(500);
  double minf;
  cout << "Starting optimization... " << endl;
  nlopt::result result = opt.optimize(var0, minf);
  if (result < 0) {
    cout << "nlopt failed!" << endl;
  } else {
    cout << "Found minimum at the variables:" << var0[0] << endl << var0[1] << endl << var0[2] << endl << var0[3] << endl << "with f:" << endl << minf << endl;
  }

  setEndEffectorZX(var0[2]);
  Matrix4f tX, tY;
  MathsUtils::makeRotationX(tX, var0[0]);
  MathsUtils::makeRotationY(tY, var0[1]);
  impactPose.block(0, 0, 3, 3) = (tY * tX).block(0, 0, 3, 3);
  cout << "Optimized endEffector pose:\n " << endEffector << endl;
  cout << "Max Velocity in Given Direction:\n " << var0[3] << endl;
  cout << "Best EndEffector Position: " << endEffector << endl;
}

void
KickModule::plotPoint(const float& x, const float& y, const string& title,
  const unsigned& ls)
{
  gp << "replot '<echo " << x << " " << y << "' with points ls " << ls << " title '" << title << "'\n";
}

void
KickModule::plotKick()
{
  PRINT("Plotting kick parameters...")
  cout << "ballPosition: " << ballPosition << endl;
  float centerSpacing = fabsf(supportToKick(1, 3) / 2);
  float offset = supportLeg == CHAIN_L_LEG ? centerSpacing : -centerSpacing;
  cout << "offset: " << offset << endl;
  cout << "ballPosition-offset: " << ballPosition[1] - offset << endl;
  auto eeTrans = supportToKick * endEffector;
  for (int i = 0; i < cPoses.size(); ++i)
    plotPoint(-(cPoses[i](1, 3) + offset), cPoses[i](0, 3), "", 1);

  gp << "set object 1 circle at " << -(ballPosition[1] + offset) << "," << ballPosition[0] << " size 0.05 fs transparent solid 0.35 fc rgb 'red'\n";
  string contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveLeftXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";
  contourLog = ConfigManager::getLogsDirPath() + string("KickModule/FootCurveRightXY.txt");
  gp << "replot '" << contourLog << "' using 3:2 with lines title '' lc rgb 'red'\n";

  if (supportLeg == CHAIN_R_LEG) {
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << centerSpacing << "," << 0 << " to " << centerSpacing - 0.025 << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  } else {
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -centerSpacing << "," << 0.025 << " as 1 lc rgb 'blue'\n"; // Support frame
    gp << "set arrow from " << -centerSpacing << "," << 0 << " to " << -(centerSpacing + 0.025) << "," << 0 << " as 1 lc rgb 'blue'\n"; // Support frame
  }
  gp << "replot\n";
  cin.get();
}

void
KickModule::plotJointTrajectories()
{
  Gnuplot gp;
  vector < pair<float, float> > times_pos;
  gp << "set xrange [0:20]\nset yrange [0:20]\n";
  gp << "plot" << gp.file1d(times_pos) << "with lines title 'Joint Trajectories'" << endl;
  int chainSize = kM->getChainSize(kickLeg);
  for (int i = 1; i < chainSize; ++i) {
    int trajStep = 0;
    while (true) {
      gp << "set terminal wxt " << i << endl;
      float pos = jointTrajectories[i][trajStep];
      float t = (trajStep + 1) * cycleTime;
      times_pos.push_back(make_pair(t, pos));
      ++trajStep;
      if (trajStep == jointTrajectories[0].size()) break;
    }
    gp << "plot" << gp.file1d(times_pos) << "with lines title 'joint Trajectories " << i << " position.'" << endl;
    cin.get();
  }
}
*/
//void
//KickModule::virtualMassKick()
//{
  //float staticFriction = 0.61, rollingFriction = 0.022, vMass;
  /*Redesign cPoses.push_back(supportToKick * endEffector);
   cPoses[0].block(0, 0, 3, 3) = Matrix3f::Identity();
   Matrix4f interPose = cPoses[0];
   interPose(0, 3) = interPose(0, 3) + -0.01;
   interPose(1, 3) = interPose(1, 3) - 0.02;
   interPose(2, 3) = interPose(2, 3) + 0.03;
   cPoses.push_back(interPose);
   // Matrix4f t;
   //MathsUtils::makeRotationY(t, 0 * M_PI / 180);
   // retractionPose.block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
   //cPoses.push_back(retractionPose);
   //  MathsUtils::makeRotationY(t, -0 * M_PI / 180);
   // impactPose.block(0, 0, 3, 3) = t.block(0, 0, 3, 3);
   // cPoses.push_back(impactPose);
   cout << "endEffector: " << endl << cPoses[0] << endl;
   cout << "interPose: " << endl << cPoses[1] << endl;
   cout << "retractionPose: " << endl << cPoses[2]<< endl;
   cout << "impactPose: " << endl << cPoses[3] << endl;
   Matrix4f torsoToSLeg =
   kM->getForwardEffector(KinematicsModule::ACTUAL, CHAIN_L_LEG, ANKLE);
   cout << "torsoToSLeg: " << endl << torsoToSLeg << endl;
   for (int i = 0; i < cPoses.size(); ++i)
   cPoses[i] = torsoToSLeg * cPoses[i];
   cout << "endEffector2: " << endl << cPoses[0] << endl;
   cout << "interPose2: " << endl << cPoses[1] << endl;
   cout << "retractionPose2: " << endl << cPoses[2]<< endl;
   cout << "impactPose2: " << endl << cPoses[3] << endl; Redesign*/
  // Calculated formula for ball dynamics based on carpet friction
  //float desBallVel = sqrt(
  //  targetDistance / (0.026 / rollingFriction + 0.02496 / staticFriction));

  /*vector<VectorXf> fAngles;
   if (kickLeg == CHAIN_L_LEG)
   fAngles = kM->inverseLeftLeg(endEffector, cPoses.back());
   else
   fAngles = kM->inverseRightLeg(endEffector, cPoses.back());

   if (fAngles.size() != 0) {
   kM->setChainPositions(KinematicsModule::SIM, kickLeg, fAngles[0]);
   } else {
   ERROR("KickModule: Failed to solve ik for final pose.")
   inBehavior = false;
   return;
   }
   cout << "endEffector:" << endEffector << endl;
   bool vmFound =
   kM->computeVirtualMass(
   KinematicsModule::SIM,
   kickLeg,
   ballToTargetUnit,
   endEffector,
   vMass
   );
   if (!vmFound) {
   ERROR("Virtual mass not computed successfully.")
   inBehavior = false;
   return;
   }
   cout << "desBallVel: " << desBallVel  << endl;
   float desEndEffVel = 0.0; //desBallVel * (vMass + 0.044) /(2 * vMass);
   cout << "desEndEffVel: " << desEndEffVel << endl;
   VectorXf iVel, fVel;
   iVel.resize(6);
   fVel.resize(6);
   iVel.setZero();
   fVel.setZero();
   fVel[0] = desEndEffVel * ballToTargetUnit[0];
   fVel[1] = desEndEffVel * ballToTargetUnit[1];
   fVel[2] = 0.0;
   iVel.block(0, 0, 3, 1) = torsoToSLeg.block(0, 0, 3, 3) * iVel.segment(0, 3);
   fVel.block(0, 0, 3, 1) = torsoToSLeg.block(0, 0, 3, 3) * fVel.segment(0, 3);
   cBoundVels.push_back(iVel);
   cBoundVels.push_back(fVel);
   trajectoryPlanner->cartesianPlanner(
   jointTrajectories,
   CHAIN_R_LEG,
   endEffector,
   cPoses,
   cBoundVels
   );Redesign*/
 /* PRINT("Starting VM kick design")
  float ballMass = 0.044;
  string baseLeg;
  MatrixXf rKnots, kKnots, totalJointConfigs, jointsTraj;
  VectorXf finalConfig;
  finalConfig.resize(5);
  int rSteps = 1, kSteps = 2;
  VectorXf jointAngles;
  jointAngles.resize(R_LEG_SIZE * 2);
  Vector3f retractEndEff = Vector3f(
    endEffector(0, 3) / 2,
    endEffector(1, 3) / 2,
    endEffector(2, 3) / 2);
  Vector3f endEff = Vector3f(
    endEffector(0, 3),
    endEffector(1, 3),
    endEffector(2, 3));
  VectorXf finalJointV, initialJointV;
  initialJointV.resize(R_LEG_SIZE - 1);
  initialJointV.setZero();
  finalJointV.resize(R_LEG_SIZE - 1);
  finalJointV.setZero();
  if (kickLeg == CHAIN_L_LEG) {
    baseLeg = "RLeg";
    jointAngles[0] = 0;
    jointAngles[1] = 9 * M_PI / 180;
    jointAngles[2] = -40 * M_PI / 180;
    jointAngles[3] = 80 * M_PI / 180;
    jointAngles[4] = -40 * M_PI / 180;
    jointAngles[5] = -18 * M_PI / 180;
    jointAngles[6] = 0;
    jointAngles[7] = 9 * M_PI / 180;
    jointAngles[8] = -32.5 * M_PI / 180;
    jointAngles[9] = 65 * M_PI / 180;
    jointAngles[10] = -32.5 * M_PI / 180;
    jointAngles[11] = -18 * M_PI / 180;
  } else {
    baseLeg = "LLeg";
    jointAngles[0] = 0;
    jointAngles[1] = -9 * M_PI / 180;
    jointAngles[2] = -40 * M_PI / 180;
    jointAngles[3] = 80 * M_PI / 180;
    jointAngles[4] = -40 * M_PI / 180;
    jointAngles[5] = 18 * M_PI / 180;
    jointAngles[6] = 0;
    jointAngles[7] = -9 * M_PI / 180;
    jointAngles[8] = -32.5 * M_PI / 180; //-35 * M_PI / 180;
    jointAngles[9] = 65 * M_PI / 180; //70 * M_PI / 180;
    jointAngles[10] = -32.5 * M_PI / 180; //-35 * M_PI / 180;
    jointAngles[11] = 18 * M_PI / 180;
  }
  PRINT("Initial configurations set")
  //cout << "baseLeg:" << baseLeg << endl;
  //cout << "endEF:" << endEff << endl;
  //cout << "jointAngles:" << jointAngles << endl;
  Vector3f initialPosition = forwardKinematics(endEff, baseLeg, jointAngles);
  cout << "initialPosition: " << initialPosition << endl;
  cout << "retractionPose: " << retractionPose << endl;
  rKnots.resize(2, 3);
  rKnots(0, 0) = initialPosition[0];
  rKnots(0, 1) = initialPosition[1];
  rKnots(0, 2) = initialPosition[2];
  rKnots(1, 0) = retractionPose(0, 3);
  rKnots(1, 1) = retractionPose(1, 3);
  rKnots(1, 2) = retractionPose(2, 3);
  cout << "RetractionKnots: " << endl << rKnots << endl;
  kKnots.resize(3, 3);
  kKnots(0, 0) = rKnots(1, 0);
  kKnots(0, 1) = rKnots(1, 1);
  kKnots(0, 2) = rKnots(1, 2);
  kKnots(1, 0) = impactPose(0, 3);
  kKnots(1, 1) = impactPose(1, 3);
  kKnots(1, 2) = impactPose(2, 3);
  kKnots(2, 0) = impactPose(0, 3) + ballRadius * ballToTargetUnit[0];
  kKnots(2, 1) = impactPose(1, 3) + ballRadius * ballToTargetUnit[1];
  kKnots(2, 2) = impactPose(2, 3) + 0.01;
  cout << "KickKnots: " << endl << kKnots << endl;
  totalJointConfigs.resize(rSteps + kSteps + 1, 5);
  Vector3f knots;
  string state;
  VectorXf finalEndEffVel;
  finalEndEffVel.resize(6);
  finalEndEffVel.setZero();
  VectorXf chainAngles;
  for (int i = 0; i < rSteps + kSteps + 1; ++i) {
    if (i <= rSteps) {
      //cout << "In retraction plan" << endl;
      state = "Retraction";
      knots[0] = rKnots(i, 0);
      knots[1] = rKnots(i, 1);
      knots[2] = rKnots(i, 2);
      initialPosition = forwardKinematics(endEff, baseLeg, jointAngles);
      //cout << "initialPosition: " << initialPosition << endl;
      bool success = inverseKinematics(
        state,
        initialPosition,
        knots,
        retractEndEff,
        jointAngles);
      //cout << "sucess: " << success << endl;
      if (!success) return;
    } else if (i > rSteps && i <= (rSteps + kSteps)) {
      //cout << "In kick plan1" << endl;
      knots[0] = kKnots(i - rSteps, 0);
      knots[1] = kKnots(i - rSteps, 1);
      knots[2] = kKnots(i - rSteps, 2);
      initialPosition = forwardKinematics(endEff, baseLeg, jointAngles);
      //cout << "In kick plan2" << endl;
      if ((kSteps + rSteps - i) == 1) {
        MatrixXf JWinv, Jinv, JVinv;
        bool increaseZVel = false;
        do {
          //cout << "In kick final" << endl;
          if (increaseZVel) {
            finalEndEffVel(2, 0) = finalEndEffVel(2, 0) + 0.05;
          } else {
            while (true) {
              initialPosition = forwardKinematics(endEff, baseLeg, jointAngles);
              bool success = inverseKinematics(
                "Retraction",
                initialPosition,
                knots,
                endEff,
                jointAngles);
              if (!success) return;
              break;
              Vector3f newPosition = forwardKinematics(
                endEff,
                baseLeg,
                jointAngles);
              Vector3f dX = newPosition - knots;
              Vector3f absDx = dX.cwiseAbs();
              if (absDx[0] < 0.0005 && absDx[1] < 0.0005 && absDx[2] < 0.0005) break;
            }
            //cout << "In kick final" << endl;
            //JointAngles = InverseKinematics(State, kickLeg, InitialPosition, Knots, EE, JointAngles);
            if (kickLeg == CHAIN_L_LEG) {
              finalConfig[0] = jointAngles[7];
              finalConfig[1] = jointAngles[8];
              finalConfig[2] = jointAngles[9];
              finalConfig[3] = jointAngles[10];
              finalConfig[4] = jointAngles[11];
              chainAngles = jointAngles.segment(6, 6);
            } else {
              finalConfig[0] = jointAngles[1];
              finalConfig[1] = jointAngles[2];
              finalConfig[2] = jointAngles[3];
              finalConfig[3] = jointAngles[4];
              finalConfig[4] = jointAngles[5];
              chainAngles = jointAngles.segment(0, 6);
            }
            //cout << "In vm calcualtion" << endl;
            vMass = calcVirtualMass(jointAngles, endEff, ballToTargetUnit);
            cout << "desBallVel: " << desBallVel << endl;
            float desEndEffVel = desBallVel * (vMass + ballMass) / (2 * vMass);
            //if (kickCount > 0)
            desEndEffVel = 0.8f;
            //else 
            // desEndEffVel = 0.5f;
            //cin >> desEndEffVel;
            //cout << "FinalV: " << (2 * vMass) / (vMass + 0.055) * finalEEV << endl;
            finalEndEffVel(0, 0) = desEndEffVel * ballToTargetUnit[0];
            finalEndEffVel(1, 0) = desEndEffVel * ballToTargetUnit[1];
            finalEndEffVel(2, 0) = desEndEffVel * ballToTargetUnit[2];
            finalEndEffVel(3, 0) = 0;
            finalEndEffVel(4, 0) = 0;
            finalEndEffVel(5, 0) = 0;
            MatrixXf JV = KickUtils::EEJacobian(
              kickLeg,
              endEff,
              finalConfig,
              "Velocity");
            //MatrixXf tJV = JV.transpose();
            JVinv = MathsUtils::pseudoInverse(JV);
            //MatrixXf JW = KickUtils::EEJacobian(kickLeg, endEff, finalConfig, "AVelocity");
            //MatrixXf tJW = JW.transpose();
            //JWinv = tJW * (JW * tJW).inverse();
            cout << "prevJV" << endl << JV << endl;
            //cout << "prevJW" << endl << JW << endl;
            cout << "Jinv" << JVinv << endl;
            //cout << "Jinv" << JWinv << endl;
            Jinv.resize(R_LEG_SIZE - 1, 3);
            Jinv = JVinv;
            //Jinv << JVinv(0,0), JVinv(0,1), JVinv(0,2), JWinv(0,0), JWinv(0,1), JWinv(0,2),
            // JVinv(1,0), JVinv(1,1), JVinv(1,2), JWinv(1,0), JWinv(1,1), JWinv(1,2),
             //JVinv(2,0), JVinv(2,1), JVinv(2,2), JWinv(2,0), JWinv(2,1), JWinv(2,2),
             //JVinv(3,0), JVinv(3,1), JVinv(3,2), JWinv(3,0), JWinv(3,1), JWinv(3,2),
             //JVinv(4,0), JVinv(4,1), JVinv(4,2), JWinv(4,0), JWinv(4,1), JWinv(4,2);
             //increaseZVel = true;
            //cout << " FinalEEV: " << JV * finalJointV << endl;
          }
          //if(isnan(JWinv(0,0)))
          //  finalJointV =  JVinv*finalEndEffVel.block(0,0,3,1);
          //else
          finalJointV = Jinv * finalEndEffVel.block(0, 0, 3, 1);
          int chainStart = kM->getChainStart(kickLeg);
          int chainSize = kM->getChainSize(kickLeg);
          Vector4f ee;
          ee << endEff, 1;
          kM->setJointPositions(chainStart, chainAngles, KinematicsModule::SIM);
          cout << "chainAngles: " << chainAngles << endl;
          MatrixXf JX = kM->computeLimbJ(kickLeg, ee, KinematicsModule::SIM);
          cout << "JX" << endl << JX << endl;
          MatrixXf jjj = KickUtils::EEJacobian(
            kickLeg,
            endEff,
            finalConfig,
            "Complete");
          cout << "Velocity remapped" << endl << jjj * finalJointV << endl;
          MatrixXf JXinv = MathsUtils::pseudoInverse(JX);
          cout << "JXinv" << endl << JXinv << endl;
          cout << " FinalJointV:  " << finalJointV << endl;
          cout << " FinalEEV: " << finalEndEffVel << endl;
          cout << " Vmass: " << vMass << endl;
        } while (abs(finalJointV(0, 0)) >= 4.09 || abs(finalJointV(1, 0)) >= 6.39 || abs(
          finalJointV(2, 0)) >= 6.39 || abs(finalJointV(3, 0)) >= 6.39 || abs(
          finalJointV(4, 0)) >= 4.09);
      } else {
        //cout << "In intermediate plan" << endl<< endl;
        //cout << "state: " << endl<< state << endl;
        //cout << "initialPosition: " << endl<< initialPosition << endl;
        //cout << "knots: " << endl<< knots << endl;
        //cout << "endEff: " << endl<<  endEff << endl;
        //cout << "jointAngles" << endl << jointAngles << endl;
        bool success = inverseKinematics(
          "Retraction",
          initialPosition,
          knots,
          endEff,
          jointAngles);
        if (!success) return;
      }
    }
    if (kickLeg == CHAIN_L_LEG) {
      totalJointConfigs(i, 0) = jointAngles[7];
      totalJointConfigs(i, 1) = jointAngles[8];
      totalJointConfigs(i, 2) = jointAngles[9];
      totalJointConfigs(i, 3) = jointAngles[10];
      totalJointConfigs(i, 4) = jointAngles[11];
    } else {
      totalJointConfigs(i, 0) = jointAngles[1];
      totalJointConfigs(i, 1) = jointAngles[2];
      totalJointConfigs(i, 2) = jointAngles[3];
      totalJointConfigs(i, 3) = jointAngles[4];
      totalJointConfigs(i, 4) = jointAngles[5];
    }
  }
  //cout << "totalJointConfigs: " << totalJointConfigs << endl;
  cout << "initialJointV: " << initialJointV << endl;
  cout << "finalJointV: " << finalJointV << endl;
  jointsTraj = vmKickPlanner(totalJointConfigs, initialJointV, finalJointV);
  //cout << "jointsTraj: " << jointsTraj << endl;*/
  /*for(int i = 1 ; i< jointsTraj.rows(); ++i)
   {
   if((jointsTraj(i, 0) - jointsTraj(i-1,0)) > 0.0401)
   jointsTraj(i, 0) = jointsTraj(i-1,0) + 0.0401;
   else if((jointsTraj(i, 0) - jointsTraj(i-1,0)) < -0.0401)
   jointsTraj(i, 0) = jointsTraj(i-1,0) - 0.0401;
   if(abs(jointsTraj(i,1) - jointsTraj(i-1,1)) > 0.064)
   jointsTraj(i,1) = jointsTraj(i-1,1) + 0.064;
   else if((jointsTraj(i,1) - jointsTraj(i-1,1)) < -0.064)
   jointsTraj(i,1) = jointsTraj(i-1,1) - 0.064;
   if(abs(jointsTraj(i,2) - jointsTraj(i-1,2)) > 0.064)
   jointsTraj(i,2) = jointsTraj(i-1,2) + 0.064;
   else if((jointsTraj(i,2) - jointsTraj(i-1,2)) < -0.064)
   jointsTraj(i,2) = jointsTraj(i-1,2) - 0.064;
   if(abs(jointsTraj(i,3) - jointsTraj(i-1,3)) > 0.064)
   jointsTraj(i,3) = jointsTraj(i-1,3) + 0.064;
   else if((jointsTraj(i,3) - jointsTraj(i-1,3)) < -0.064)
   jointsTraj(i,3) = jointsTraj(i-1,3) - 0.064;
   if(abs(jointsTraj(i,4) - jointsTraj(i-1,4)) > 0.0401)
   jointsTraj(i,4) = jointsTraj(i-1,4) + 0.0401;
   else if((jointsTraj(i,4) - jointsTraj(i-1,4)) < -0.0401)
   jointsTraj(i,4) = jointsTraj(i-1,4) - 0.0401;
   }*/
 /* cout << jointsTraj * 180 / M_PI<< endl;
  vector < vector<float> > trajs(R_LEG_SIZE - 1);
  for (int i = 0; i < R_LEG_SIZE - 1; ++i) {
    for (int j = 0; j < jointsTraj.rows() - 15; ++j) {
      trajs[i].push_back(jointsTraj(j, i));
    }
  }
  jointTrajectories = trajs;
}*/
/*
MatrixXf
KickModule::vmKickPlanner(const MatrixXf& jointPoses,
  const MatrixXf& initialJointV, const MatrixXf& finalJointV)
{
  MatrixXf A, b, jointA, jointV, spline;
  float tStep = 0.25, t0 = 0, tf = 0.25;
  int n = jointPoses.rows();
  int nJoints = jointPoses.cols();
  VectorXf knots, time;
  knots.resize(n - 1);
  time.resize(n);
  A.resize(n, n);
  A.setZero();
  b.resize(n, nJoints);
  b.setZero();
  jointA.resize(n, nJoints);
  jointA.setZero();
  jointV.resize(n, nJoints);
  jointV.setZero();
  spline.resize((n - 1) * (tStep * 100), nJoints);
  for (int i = 0; i < n; ++i)
    time[i] = i * tStep;
  for (int i = 0; i < n - 1; ++i)
    knots[i] = tStep;

  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < 5; ++j) {
      if (i == 0) jointV(i, j) = initialJointV(j, 0);
      else if (i == n - 1) jointV(i, j) = 0;
      else if (i == n - 2) jointV(i, j) = finalJointV(j, 0);
      else jointV(i, j) =
        -knots[i] / 2 * jointA(i, j) - (jointPoses(i + 1, j) - jointPoses(i, j)) / knots[i] + (knots[i] * (jointA(
          i + 1,
          j) + jointA(i, j))) / 6;
    }
  }
  MatrixXf cMat(6, 6), coeffsJ1(6, n - 1), coeffsJ2(6, n - 1), coeffsJ3(
    6,
    n - 1), coeffsJ4(6, n - 1), coeffsJ5(6, n - 1), stateMat(6, 1);
  cMat << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5), 0, 1, 2 * t0, 3 * pow(
    t0,
    2), 4 * pow(t0, 3), 5 * pow(t0, 4), 0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(
    t0,
    3), 1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5), 0, 1, 2 * tf, 3 * pow(
    tf,
    2), 4 * pow(tf, 3), 5 * pow(tf, 4), 0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(
    tf,
    3);
  for (int i = 0; i < n - 1; ++i) {
    stateMat << jointPoses(i, 0), jointV(i, 0), jointA(i, 0), jointPoses(
      i + 1,
      0), jointV(i + 1, 0), jointA(i + 1, 0);
    coeffsJ1.block(0, i, 6, 1) = cMat.inverse() * stateMat;
    stateMat << jointPoses(i, 1), jointV(i, 1), jointA(i, 1), jointPoses(
      i + 1,
      1), jointV(i + 1, 1), jointA(i + 1, 1);
    coeffsJ2.block(0, i, 6, 1) = cMat.inverse() * stateMat;
    stateMat << jointPoses(i, 2), jointV(i, 2), jointA(i, 2), jointPoses(
      i + 1,
      2), jointV(i + 1, 2), jointA(i + 1, 2);
    coeffsJ3.block(0, i, 6, 1) = cMat.inverse() * stateMat;
    stateMat << jointPoses(i, 3), jointV(i, 3), jointA(i, 3), jointPoses(
      i + 1,
      3), jointV(i + 1, 3), jointA(i + 1, 3);
    coeffsJ4.block(0, i, 6, 1) = cMat.inverse() * stateMat;
    stateMat << jointPoses(i, 4), jointV(i, 4), jointA(i, 4), jointPoses(
      i + 1,
      4), jointV(i + 1, 4), jointA(i + 1, 4);
    coeffsJ5.block(0, i, 6, 1) = cMat.inverse() * stateMat;
  }
  int k = 0;
  for (int i = 0; i < n - 1; ++i) {
    float j = 0;
    for (int m = 0; m < 25; ++m) {
      j = j + knots[i] / 25;
      spline(k, 0) = coeffsJ1(0, i) + coeffsJ1(1, i) * j + coeffsJ1(2, i) * pow(
        j,
        2) + coeffsJ1(3, i) * pow(j, 3) + coeffsJ1(4, i) * pow(j, 4) + coeffsJ1(
        5,
        i) * pow(j, 5);
      spline(k, 1) = coeffsJ2(0, i) + coeffsJ2(1, i) * j + coeffsJ2(2, i) * pow(
        j,
        2) + coeffsJ2(3, i) * pow(j, 3) + coeffsJ2(4, i) * pow(j, 4) + coeffsJ2(
        5,
        i) * pow(j, 5);
      spline(k, 2) = coeffsJ3(0, i) + coeffsJ3(1, i) * j + coeffsJ3(2, i) * pow(
        j,
        2) + coeffsJ3(3, i) * pow(j, 3) + coeffsJ3(4, i) * pow(j, 4) + coeffsJ3(
        5,
        i) * pow(j, 5);
      spline(k, 3) = coeffsJ4(0, i) + coeffsJ4(1, i) * j + coeffsJ4(2, i) * pow(
        j,
        2) + coeffsJ4(3, i) * pow(j, 3) + coeffsJ4(4, i) * pow(j, 4) + coeffsJ4(
        5,
        i) * pow(j, 5);
      spline(k, 4) = coeffsJ5(0, i) + coeffsJ5(1, i) * j + coeffsJ5(2, i) * pow(
        j,
        2) + coeffsJ5(3, i) * pow(j, 3) + coeffsJ5(4, i) * pow(j, 4) + coeffsJ5(
        5,
        i) * pow(j, 5);
      k = k + 1;
    }
  }
  return spline;
}*/
/*
float
KickModule::calcVirtualMass(const VectorXf& jointAngles, const Vector3f& endEff,
  const Vector3f& direction)
{
  VectorXf chainAngles;
  VectorXf configAngles;
  Vector3f com;
  Matrix3f R, G11, G12, G22, M;
  configAngles.resize(R_LEG_SIZE - 1);
  chainAngles.resize(R_LEG_SIZE);
  MatrixXf MM, J, G;
  string baseLeg;
  float vMass;
  if (kickLeg == CHAIN_L_LEG) {
    com[0] = 0.02542; //Centre of Mass Coordinates of Left Leg
    com[1] = 0.003300;
    com[2] = -0.03239;
    baseLeg = "RLeg";
    configAngles[0] = jointAngles[7];
    configAngles[1] = jointAngles[8];
    configAngles[2] = jointAngles[9];
    configAngles[3] = jointAngles[10];
    configAngles[4] = jointAngles[11];
    chainAngles = jointAngles.segment(6, 6);
  } else {
    com[0] = 0.02540; //Centre of Mass Coordinates of Right Leg
    com[1] = -0.00332;
    com[2] = -0.03239;
    baseLeg = "LLeg";
    configAngles[0] = jointAngles[1];
    configAngles[1] = jointAngles[2];
    configAngles[2] = jointAngles[3];
    configAngles[3] = jointAngles[4];
    configAngles[4] = jointAngles[5];
    chainAngles = jointAngles.segment(0, 6);
  }
  cout << "jointAngles: " << chainAngles << endl;
  cout << "jointAngles: " << configAngles << endl;
  int chainStart = kM->getChainStart(kickLeg);
  int chainSize = kM->getChainSize(kickLeg);
  kM->setJointPositions(chainStart, chainAngles, KinematicsModule::SIM);
  cout << "Mass matrix new: " << kM->computeMassMatrix(
    kickLeg,
    KinematicsModule::SIM) << endl;

  //cout << "finalBB" << B << endl;
  //InertiaMatrix (5x5) at Final Config for Kicking Leg
  MM = KickUtils::calcMassMatrix(kickLeg, configAngles);

  cout << "Mass matrix: " << endl << MM << endl;

  //Jacobian (6x5) at Final Config for Kicking Leg
  J = KickUtils::EEJacobian(kickLeg, endEff, configAngles, "Complete");

  cout << "Jacobian: " << endl << J << endl;

  //InertiaMatrix inverse (5x5) at Final Config for Kicking Leg
  MM = MM.inverse();

  cout << "Inverse mass matrix: " << endl << MM << endl;

  //Inertial Projection in Cartesian Space (6x6) at Final Config for Kicking Leg  (G = 6x6 Symmetric) [G11, G12;G21, G22]
  G = J * MM * J.transpose();

  cout << "G matrix: " << endl << G << endl;

  //Distance of Contact Point from the point at which InertiaMatrix is determined i.e. the COM of Ankle
  //R = 3x3 Skew Symmetric Matrix [0, Rz, -Ry;-Rz, 0, Rx;Ry, -Rx, 0]
  R = MathsUtils::makeSkewMat(endEff - com);
  //cout << "R" << R << endl;
  //Translational Component of InertiaMatrix in Cartesian Space
  G11 << G(0, 0), G(0, 1), G(0, 2), G(1, 0), G(1, 1), G(1, 2), G(2, 0), G(2, 1), G(
    2,
    2);
  //Translational + Rotational Component of InertiaMatrix in Cartesian Space (G = 6x6 Symmetric hence G12 = G21)
  G12 << G(0, 3), G(0, 4), G(0, 5), G(1, 3), G(1, 4), G(1, 5), G(2, 3), G(2, 4), G(
    2,
    5);
  //Rotational Component of InertiaMatrix in Cartesian Space
  G22 << G(3, 3), G(3, 4), G(3, 5), G(4, 3), G(4, 4), G(4, 5), G(5, 3), G(5, 4), G(
    5,
    5);

  //Inertia Matrix in cartesian space conversion from COM to contact point using 'R' for transformation
  //M is (3x3) Matrix concerned with translation of the contact point while G11 was concerned with translation of COM of Ankle
  M = G11 + R * G12.transpose() + G12 * R.transpose() + R * G22 * R.transpose();

  cout << "M matrix in cartesian space: " << endl << M << endl;
  //cout << "M" << M << endl;
  //Virtual Mass in the target direction  = 1/(Direction^t*M*Direction)
  //VectorXf d;
  //d.resize(6);
  //d[0] = 1;
  //d[4] = 1;
  //vMass = d.transpose() * G * d;
  vMass = direction.transpose() * M * direction;
  cout << "vMass" << vMass << endl;
  vMass = 1 / vMass;

  cout << "vMass" << vMass << endl;
  return vMass;
}

bool
KickModule::inverseKinematics(const string& state,
  const Vector3f& initialPosition, const Vector3f& finalPosition,
  const Vector3f& endEff, VectorXf& jointAngles)
{
  VectorXf jointAnglesI(5), res(5);
  Vector3f iPinBaseFrame, fPinBaseFrame, dX, error;
  Vector3f ip = initialPosition;
  int iterations = 0;
  MatrixXf J(3, 5), tJ(5, 3), JJt(3, 3), InvJJt(5, 5), Jinv(5, 3);
  if (kickLeg == CHAIN_L_LEG) {
    iPinBaseFrame = forwardKinematics(ip, "LLegBase", jointAngles);
    fPinBaseFrame = forwardKinematics(finalPosition, "LLegBase", jointAngles);
  } else {
    //cout << "FinalPosition: " << finalPosition << endl;
    //cout << "JointAngles: " << jointAngles << endl;
    iPinBaseFrame = forwardKinematics(ip, "RLegBase", jointAngles);
    fPinBaseFrame = forwardKinematics(finalPosition, "RLegBase", jointAngles);
    // cout << "------------------fPinBaseFrame: --------------" << endl << fPinBaseFrame << endl;
  }
  string baseLeg;
  if (kickLeg == CHAIN_L_LEG) baseLeg = "RLeg";
  else baseLeg = "LLeg";

  dX = fPinBaseFrame - iPinBaseFrame;
  Vector3f absDx = dX.cwiseAbs();
  // cout << "iPinBaseFrame: " << iPinBaseFrame << endl;
  // cout << "fPinBaseFrame: " << fPinBaseFrame << endl;
  // cout << "endeff: " << endEff << endl;
  while (absDx[0] > 0.0005 || absDx[1] > 0.0005 || absDx[2] > 0.0005) {
    //cout << "iterations: " << iterations << endl;
    if (iterations > 50) {
      ERROR("Cannot reach given kick trajectory.")
      return false;
    }
    if (kickLeg == CHAIN_L_LEG) {
      jointAnglesI[0] = jointAngles[7];
      jointAnglesI[1] = jointAngles[8];
      jointAnglesI[2] = jointAngles[9];
      jointAnglesI[3] = jointAngles[10];
      jointAnglesI[4] = jointAngles[11];
    } else {
      jointAnglesI[0] = jointAngles[1];
      jointAnglesI[1] = jointAngles[2];
      jointAnglesI[2] = jointAngles[3];
      jointAnglesI[3] = jointAngles[4];
      jointAnglesI[4] = jointAngles[5];
    }
    J = KickUtils::EEJacobian(kickLeg, endEff, jointAnglesI, "Velocity");
    //if (iterations == 0)
    //  cout << "J: " << J << endl;
    tJ = J.transpose();
    JJt = (J * tJ);
    InvJJt = JJt.inverse();
    Jinv = tJ * InvJJt;
    error = (Matrix3f::Identity() - J * Jinv) * dX;
    Vector3f absError = error.cwiseAbs();
    //if (iterations == 0)
    //  cout << "error: " << error << endl;

    //if (iterations == 0)
    //    cout << "Jinv: " << Jinv << endl;

    while (absError[0] > 0.01 || absError[1] > 0.01 || absError[2] > 0.01) {
      //if (iterations == 0)
      //  cout << "dX: " << dX << endl;
      dX = dX / 2;
      error = (Matrix3f::Identity() - J * Jinv) * dX;
      absError = error.cwiseAbs();
    }
    res = Jinv * dX;
    ///if (iterations == 0)
    //  cout << "res: " << res << endl;
    jointAnglesI = jointAnglesI + res;
    if (kickLeg == CHAIN_L_LEG) {
      if (jointAnglesI[0] < -20.97 * M_PI / 180) {
        jointAnglesI[0] = (-20.97 * M_PI / 180);
      } else if (jointAnglesI[0] > (45.29 * M_PI / 180)) {
        jointAnglesI[0] = (45.29 * M_PI / 180);
      }

      if (jointAnglesI[1] < (-88 * M_PI / 180)) {
        jointAnglesI[1] = (-88 * M_PI / 180);
      } else if (jointAnglesI[1] > (27.73 * M_PI / 180)) {
        jointAnglesI[1] = (27.73 * M_PI / 180);
      }

      if (jointAnglesI[2] < (-5.29 * M_PI / 180)) {
        jointAnglesI[2] = (-5.29 * M_PI / 180);
      } else if (jointAnglesI[2] > (121.04 * M_PI / 180)) {
        jointAnglesI[2] = (121.04 * M_PI / 180);
      }

      if (jointAnglesI[3] < (-53 * M_PI / 180)) {
        jointAnglesI[3] = (-53 * M_PI / 180);
      } else if (jointAnglesI[3] > (68.15 * M_PI / 180)) {
        jointAnglesI[3] = (68.15 * M_PI / 180);
      }

      if (jointAnglesI[4] < (-44.06 * M_PI / 180)) {
        jointAnglesI[4] = (-44.06 * M_PI / 180);
      } else if (jointAnglesI[4] > (22.79 * M_PI / 180)) {
        jointAnglesI[4] = (22.79 * M_PI / 180);
      }
      if (state == "Retraction") {
        jointAnglesI[3] = jointAngles[10];
        jointAnglesI[4] = jointAngles[11];
      } else if (state == "COMSHIFT") {
        jointAnglesI[3] = -jointAnglesI[1] - jointAnglesI[2];
        jointAnglesI[4] = -jointAnglesI[0];
      }
      jointAngles[7] = jointAnglesI[0];
      jointAngles[8] = jointAnglesI[1];
      jointAngles[9] = jointAnglesI[2];
      jointAngles[10] = jointAnglesI[3];
      jointAngles[11] = jointAnglesI[4];
    } else {
      if (jointAnglesI[0] < (-45.29 * M_PI / 180)) {
        jointAnglesI[0] = (-45.29 * M_PI / 180);
      } else if (jointAnglesI[0] > (21.74 * M_PI / 180)) {
        jointAnglesI[0] = (21.74 * M_PI / 180);
      }

      if (jointAnglesI[1] < (-88 * M_PI / 180)) {
        jointAnglesI[1] = (-88 * M_PI / 180);
      } else if (jointAnglesI[1] > (27.73 * M_PI / 180)) {
        jointAnglesI[1] = (27.73 * M_PI / 180);
      }

      if (jointAnglesI[2] < (-5.90 * M_PI / 180)) {
        jointAnglesI[2] = (-5.90 * M_PI / 180);
      } else if (jointAnglesI[2] > (121.47 * M_PI / 180)) {
        jointAnglesI[2] = (121.47 * M_PI / 180);
      }

      if (jointAnglesI[3] < (-53.40 * M_PI / 180)) {
        jointAnglesI[3] = (-53.40 * M_PI / 180);
      } else if (jointAnglesI[3] > (67.97 * M_PI / 180)) {
        jointAnglesI[3] = (67.97 * M_PI / 180);
      }

      if (jointAnglesI[4] < (-27.80 * M_PI / 180)) {
        jointAnglesI[4] = (-27.80 * M_PI / 180);
      } else if (jointAnglesI[4] > (44.06 * M_PI / 180)) {
        jointAnglesI[4] = (44.06 * M_PI / 180);
      }
      if (state == "Retraction") {
        jointAnglesI[3] = jointAngles[4];
        jointAnglesI[4] = jointAngles[5];
      } else if (state == "COMSHIFT") {
        jointAnglesI[3] = -jointAnglesI[1] - jointAnglesI[2];
        jointAnglesI[4] = -jointAnglesI[0];
      }
      jointAngles[1] = jointAnglesI[0];
      jointAngles[2] = jointAnglesI[1];
      jointAngles[3] = jointAnglesI[2];
      jointAngles[4] = jointAnglesI[3];
      jointAngles[5] = jointAnglesI[4];
    }
    ip = forwardKinematics(endEff, baseLeg, jointAngles);
    if (kickLeg == CHAIN_L_LEG) {
      iPinBaseFrame = forwardKinematics(ip, "LLegBase", jointAngles);
      fPinBaseFrame = forwardKinematics(finalPosition, "LLegBase", jointAngles);
    } else {
      iPinBaseFrame = forwardKinematics(ip, "RLegBase", jointAngles);
      fPinBaseFrame = forwardKinematics(finalPosition, "RLegBase", jointAngles);
    }
    dX = fPinBaseFrame - iPinBaseFrame;
    absDx = dX.cwiseAbs();
    ++iterations;
  }
  return true;
}

Vector3f
KickModule::forwardKinematics(const Vector3f& endEffector,
  const string& baseFrame, const VectorXf& jointAngles)
{
  Matrix4f t;
  return forwardKinematics(endEffector, baseFrame, jointAngles, t);
}

Vector3f
KickModule::forwardKinematics(const Vector3f& endEffector,
  const string& baseFrame, const VectorXf& jointAngles, Matrix4f& endFrame)
{
  Matrix4f endEffectorFrame;
  Vector4f endEffectorVec, eePoint;
  endEffectorVec << endEffector[0], endEffector[1], endEffector[2], 1;
  Matrix4f LLegBasetoTorso, LHipYawPitchtoLLegBase, LHipRolltoLHipYawPitch,
    LHipPitchtoLHipRoll, LKneePitchtoLHipPitch, LAnklePitchtoLKneePitch,
    LAnkleRolltoLAnklePitch, LHipYawPitchtoTorso, LHipRolltoTorso,
    LHipPitchtoTorso, LKneePitchtoTorso, LAnklePitchtoTorso, LAnkleRolltoTorso;
  Matrix4f RLegBasetoTorso, RHipYawPitchtoRLegBase, RHipRolltoRHipYawPitch,
    RHipPitchtoRHipRoll, RKneePitchtoRHipPitch, RAnklePitchtoRKneePitch,
    RAnkleRolltoRAnklePitch, RHipYawPitchtoTorso, RHipRolltoTorso,
    RHipPitchtoTorso, RKneePitchtoTorso, RAnklePitchtoTorso, RAnkleRolltoTorso;

  //FKin for Right Leg
  RLegBasetoTorso << 1, 0, 0, 0, 0, 1, 0, -hipOffsetY, 0, 0, 1, -hipOffsetZ, 0, 0, 0, 1;
  MathsUtils::makeDHTransformation(
    RHipYawPitchtoRLegBase,
    0,
    -M_PI / 4,
    0,
    jointAngles[0] - M_PI / 2);
  MathsUtils::makeDHTransformation(
    RHipRolltoRHipYawPitch,
    0,
    -M_PI / 2,
    0,
    jointAngles[1] - M_PI / 4);
  MathsUtils::makeDHTransformation(
    RHipPitchtoRHipRoll,
    0,
    M_PI / 2,
    0,
    jointAngles[2] + M_PI);
  MathsUtils::makeDHTransformation(
    RKneePitchtoRHipPitch,
    thighLength,
    0,
    0,
    jointAngles[3]);
  MathsUtils::makeDHTransformation(
    RAnklePitchtoRKneePitch,
    tibiaLength,
    0,
    0,
    jointAngles[4]);
  MathsUtils::makeDHTransformation(
    RAnkleRolltoRAnklePitch,
    0,
    M_PI / 2,
    0,
    jointAngles[5]);
  RHipYawPitchtoTorso = RLegBasetoTorso * RHipYawPitchtoRLegBase;
  RHipRolltoTorso =
    RLegBasetoTorso * RHipYawPitchtoRLegBase * RHipRolltoRHipYawPitch;
  RHipPitchtoTorso =
    RLegBasetoTorso * RHipYawPitchtoRLegBase * RHipRolltoRHipYawPitch * RHipPitchtoRHipRoll;
  RKneePitchtoTorso =
    RLegBasetoTorso * RHipYawPitchtoRLegBase * RHipRolltoRHipYawPitch * RHipPitchtoRHipRoll * RKneePitchtoRHipPitch;
  RAnklePitchtoTorso =
    RLegBasetoTorso * RHipYawPitchtoRLegBase * RHipRolltoRHipYawPitch * RHipPitchtoRHipRoll * RKneePitchtoRHipPitch * RAnklePitchtoRKneePitch;
  RAnkleRolltoTorso =
    RLegBasetoTorso * RHipYawPitchtoRLegBase * RHipRolltoRHipYawPitch * RHipPitchtoRHipRoll * RKneePitchtoRHipPitch * RAnklePitchtoRKneePitch * RAnkleRolltoRAnklePitch;

  //FKin for Right Leg
  //FKin for Left Leg
  LLegBasetoTorso << 1, 0, 0, 0, 0, 1, 0, hipOffsetY, 0, 0, 1, -hipOffsetZ, 0, 0, 0, 1;
  MathsUtils::makeDHTransformation(
    LHipYawPitchtoLLegBase,
    0,
    M_PI / 4,
    0,
    jointAngles[6] + M_PI / 2);
  MathsUtils::makeDHTransformation(
    LHipRolltoLHipYawPitch,
    0,
    M_PI / 2,
    0,
    jointAngles[7] + M_PI / 4);
  MathsUtils::makeDHTransformation(
    LHipPitchtoLHipRoll,
    0,
    M_PI / 2,
    0,
    jointAngles[8] + M_PI);
  MathsUtils::makeDHTransformation(
    LKneePitchtoLHipPitch,
    thighLength,
    0,
    0,
    jointAngles[9]);
  MathsUtils::makeDHTransformation(
    LAnklePitchtoLKneePitch,
    tibiaLength,
    0,
    0,
    jointAngles[10]);
  MathsUtils::makeDHTransformation(
    LAnkleRolltoLAnklePitch,
    0,
    M_PI / 2,
    0,
    jointAngles[11]);

  LHipYawPitchtoTorso = LLegBasetoTorso * LHipYawPitchtoLLegBase;
  LHipRolltoTorso =
    LLegBasetoTorso * LHipYawPitchtoLLegBase * LHipRolltoLHipYawPitch;
  LHipPitchtoTorso =
    LLegBasetoTorso * LHipYawPitchtoLLegBase * LHipRolltoLHipYawPitch * LHipPitchtoLHipRoll;
  LKneePitchtoTorso =
    LLegBasetoTorso * LHipYawPitchtoLLegBase * LHipRolltoLHipYawPitch * LHipPitchtoLHipRoll * LKneePitchtoLHipPitch;
  LAnklePitchtoTorso =
    LLegBasetoTorso * LHipYawPitchtoLLegBase * LHipRolltoLHipYawPitch * LHipPitchtoLHipRoll * LKneePitchtoLHipPitch * LAnklePitchtoLKneePitch;
  LAnkleRolltoTorso =
    LLegBasetoTorso * LHipYawPitchtoLLegBase * LHipRolltoLHipYawPitch * LHipPitchtoLHipRoll * LKneePitchtoLHipPitch * LAnklePitchtoLKneePitch * LAnkleRolltoLAnklePitch;
  //FKin for Left Leg

  if (baseFrame == "LLegforTorso") {
    endEffectorFrame = KickUtils::rotY(M_PI / 2) * LAnkleRolltoTorso.inverse();
    endFrame = endEffectorFrame * KickUtils::rotY(-M_PI / 2);
  } else if (baseFrame == "RLegforTorso") {
    endEffectorFrame = KickUtils::rotY(M_PI / 2) * RAnkleRolltoTorso.inverse();
    endFrame = endEffectorFrame;
  } else if (baseFrame == "LLeg") {
    endEffectorFrame =
      KickUtils::rotY(M_PI / 2) * LAnkleRolltoTorso.inverse() * RAnkleRolltoTorso;
    endFrame = endEffectorFrame * KickUtils::rotY(-M_PI / 2);
  } else if (baseFrame == "RLeg") {
    endEffectorFrame =
      KickUtils::rotY(M_PI / 2) * RAnkleRolltoTorso.inverse() * LAnkleRolltoTorso;
    endFrame = endEffectorFrame * KickUtils::rotY(-M_PI / 2);
  } else if (baseFrame == "RLegBase") {
    endEffectorFrame = RLegBasetoTorso.inverse() * LAnkleRolltoTorso;
    endFrame = endEffectorFrame * KickUtils::rotY(-M_PI / 2);
  } else if (baseFrame == "LLegBase") {
    endEffectorFrame = LLegBasetoTorso.inverse() * RAnkleRolltoTorso;
    endFrame = endEffectorFrame * KickUtils::rotY(-M_PI / 2);
  } else if (baseFrame == "BASE") {
    endEffectorVec[0] = 0;
    endEffectorVec[1] = 0;
    endEffectorVec[2] = 0;
    endEffectorFrame =
      KickUtils::rotY(M_PI / 2) * LAnkleRolltoTorso.inverse() * RLegBasetoTorso;
    endFrame = endEffectorFrame;
    eePoint = endEffectorFrame * endEffectorVec;
  }
  eePoint = endFrame * endEffectorVec;
  return (Vector3f) eePoint.segment(0, 3);
}
*/
/*if (once) {
 setForwardTransform();
 setEndEffector();
 setBallHitPose();
 setRetractionPose();
 cPoses.push_back(supportToKick  * endEffector);
 cPoses.push_back(retractionPose);
 cPoses.push_back(impactPose);
 defineTrajectory();

 //trajectoryLog.open(
 //  ROOT_DIR + "ProcessingModule/MotionModule/KickModule/Logs/Trajectory.txt", std::ofstream::out | std::ofstream::trunc
 //);
 //trajectoryLog << "# Time     X     Y    Z" << endl;
 //trajectoryLog.close();
 cout << "ballToTargetUnit:" << endl << ballToTargetUnit << endl;
 cout << "targetDistance:" << endl << targetDistance << endl;
 cout << "targetAngle:" << endl << targetAngle << endl;
 cout << "kickLeg:" << endl << kickLeg << endl;
 cout << "supportToKick:" << endl << supportToKick << endl;
 cout << "endEffector:" << endl << endEffector << endl;
 cout << "RetractionPose   =   " << endl << retractionPose << endl;
 cout << "ballHitPoise   =   " << endl << impactPose << endl;
 once = false;
 }*//*
 vector<VectorXf> angles;
 Vector3f actualPosition;
 if(trajectoryPlanner->step(refPosition, refVelocity, refAcceleration)) {
 if(kickLeg == CHAIN_R_LEG) {
 setForwardTransform();
 actualPosition[0] = supportToKick(0,3);
 actualPosition[1] = supportToKick(1,3);
 actualPosition[2] = supportToKick(2,3);
 Vector3f refPositionTmp = Maths::makeTransform(baseToLegT, refPosition);
 Matrix4f targetFrame;
 Maths::makeTranslation(targetFrame, (float)refPositionTmp[0], (float)refPositionTmp[1], (float)refPositionTmp[2]);
 targetFrame.block<3,3>(0,0) = legToBaseT;
 targetFrame.block<3,3>(0,0) = (kM->getForwardEffector(KinematicsModule::ACTUAL, CHAIN_R_LEG)).block<3,3>(0,0);
 angles = kM->inverseRightLeg(targetFrame);*/
/*MatrixXf pError(3,1), res(3,1), J(3,5), tJ(5,3), JJt(3,3), InvJJt(5,5), Jinv(5,3), error(3,1);
 Matrix<float, 5, 1> jointAngles(5);
 jointAngles[0] = kM->getJointPosition(R_HIP_ROLL);
 jointAngles[1] = kM->getJointPosition(R_HIP_PITCH);
 jointAngles[2] = kM->getJointPosition(R_KNEE_PITCH);
 jointAngles[3] = kM->getJointPosition(R_ANKLE_PITCH);
 jointAngles[4] = kM->getJointPosition(R_ANKLE_ROLL);
 J = KickUtils::EEJacobian("RLeg", Maths::firstThree(endEffector), jointAngles, "Velocity");
 tJ= J.transpose();
 JJt=(J*tJ);
 InvJJt = JJt.inverse();
 Jinv = tJ*InvJJt;
 pError = refPosition - actualPosition;
 jointAngles = jointAngles + Jinv*pError;*/
/*if(angles.size() !=0) {
 cmdJoints[R_HIP_YAW_PITCH] = (angles[0])[0];
 cmdJoints[R_HIP_ROLL] = (angles[0])[1];
 cmdJoints[R_HIP_PITCH] = (angles[0])[2];
 cmdJoints[R_KNEE_PITCH] = (angles[0])[3];
 cmdJoints[R_ANKLE_PITCH] = (angles[0])[4];
 cmdJoints[R_ANKLE_ROLL] = (angles[0])[5];
 }
 else
 cout << "The desired cartesian position is out of reachable space." << endl;
 }
 else {
 setForwardTransform();
 actualPosition[0] = supportToKick(0,3);
 actualPosition[1] = supportToKick(1,3);
 actualPosition[2] = supportToKick(2,3);
 refPosition = Maths::makeTransform(baseToLegT, refPosition);
 Matrix4f targetFrame;
 Maths::makeTranslation(targetFrame, (float)refPosition[0], (float)refPosition[1], (float)refPosition[2]);
 targetFrame.block<3,3>(0,0) = legToBaseT;
 //targetFrame.block<3,3>(0,0) = (((kM->getForwardEffector((KinematicsModule::Effectors)CHAIN_L_LEG))).block<3,3>(0,0)).cast<float> ();
 angles = kM->inverseLeftLeg(targetFrame);
 if(angles.size() !=0) {
 cmdJoints[L_HIP_YAW_PITCH] = (angles[0])[0];
 cmdJoints[L_HIP_ROLL] = (angles[0])[1];
 cmdJoints[L_HIP_PITCH] = (angles[0])[2];
 cmdJoints[L_KNEE_PITCH] = (angles[0])[3];
 cmdJoints[L_ANKLE_PITCH] = (angles[0])[4];
 cmdJoints[L_ANKLE_ROLL] = (angles[0])[5];
 }
 else
 cout << "The desired cartesian position is out of reachable space." << endl;
 }
 trajectoryLog.open(ROOT_DIR + "ProcessingModule/MotionModule/KickModule/Logs/Trajectory.txt", fstream::app|fstream::out);
 trajectoryLog << trajectoryPlanner->getTrajStep()
 << "    " << actualPosition[0]
 << "    " << actualPosition[1]
 << "    " << actualPosition[2]
 << "    " << refPosition[0]
 << "    " << refPosition[1]
 << "    " << refPosition[2]
 << "    " << refVelocity[0]
 << "    " << refVelocity[1]
 << "    " << refVelocity[2];
 if(angles.size() !=0) {
 trajectoryLog  << "    " << (angles[0])[0]
 << "    " << (angles[0])[1]
 << "    " << (angles[0])[2]
 << "    " << (angles[0])[3]
 << "    " << (angles[0])[4]
 << "    " << (angles[0])[5]
 << endl;
 } else {
 trajectoryLog  << "    " << "nan"
 << "    " << "nan"
 << "    " << "nan"
 << "    " << "nan"
 << "    " << "nan"
 << "    " << "nan"
 << endl;
 }
 trajectoryLog.close();
 } else {
 this->inBehavior = false;
 cout << "InREQUEST" << inBehavior << endl;
 }*/
//  }
//  }
/*void KickModule::executeKick() {
 double time = dcmProxy->getTime(0);
 for(int i = 0; i<1000; ++i) {
 double timein = dcmProxy->getTime(0);
 kM->Update();
 balanceModule->Update();
 if(i==151)
 balanceModule->SetBalancingLegs(0);
 if(i>150 && i < 305)
 kickModule->Update();
 controlModule->ActuatorUpdate();
 double diff = dcmProxy->getTime(0) - timein;
 if( diff < 10) {
 qi::os::msleep(10 - diff);
 }
 //cout << "timeDiff = " << (dcmProxy->getTime(0) - timein) << endl;
 }
 }*/
