/**
 * @file FeatureExtraction/BallTracker.cpp
 *
 * This file implements the class BallTracker
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Jan 2018
 */

#include "VisionModule/include/FeatureExtraction/BallTracker.h"

void
BallTracker::init(const unsigned& camIndex)
{
	lost = true;
	reinitialize = true;
	this->camIndex = camIndex;
	setupKalmanFilter();
}

void 
BallTracker::reset(const unsigned& camIndex)
{
	lost = true;
	reinitialize = true;
	this->camIndex = camIndex;
	state = Mat(stateSize, 1, CV_32F);
	meas = Mat(measSize, 1, CV_32F);
}

void 
BallTracker::reset(const unsigned& camIndex, const Mat& state)
{
	//! This is used when we know the position of the ball in the given
	//! camera.
	reinitialize = true;
	this->camIndex = camIndex;
	this->state = state;
	kFilter.statePre = state;
	kFilter.statePost = state;
}

Mat
BallTracker::predict()
{
	return kFilter.predict();
}

Mat
BallTracker::getEstimatedState()
{
	if (losingBall || lost) {
		//cout << "getting statePre" << endl;
		return kFilter.statePre;
	} else {
		//cout << "getting statePost" << endl;
		return kFilter.statePost;
	}
}

void
BallTracker::updateFilter(vector<Rect> ballRect)
{
	if (ballRect.empty()) {
		//cout << "ball rect empty. Losing ball..." << endl;
		if (!losingBall) {
			timeSinceLost = 0.f;
			timeAtLost = visionModule->getModuleTime();
			losingBall = true;
		}
		kFilter.measurementNoiseCov = infMeasNoiseCov;
		kFilter.correct(meas);
	} else {
		Point posImage = Point(
		ballRect[0].x + ballRect[0].width / 2,
		ballRect[0].y + ballRect[0].height / 2);
		Point2f posWorld;
		cameraTransform->imageToWorld(camIndex, posWorld, posImage, 0.05);
		Matrix<float, 3, 4> footInCam = cameraTransform->getExtMatrix(camIndex);
		Vector4f posVec(posWorld.x, posWorld.y, 0.05, 1.f);
		Vector3f posInCamFrame = footInCam * posVec;
		//cout << "posIncamFrame: " << posInCamFrame << endl;
		//cout << "posWorld: " << posWorld << endl;
		if (posInCamFrame[2] < 0) {
			//cout << "posInCamFrame[2] < 0. Losing ball..." << endl;
			if (!losingBall) {
				timeSinceLost = 0.f;
				timeAtLost = visionModule->getModuleTime();
				losingBall = true;
			}
			kFilter.measurementNoiseCov = infMeasNoiseCov;
			kFilter.correct(meas);
		} else {
			meas.at<float>(0) = posWorld.x;
			meas.at<float>(1) = posWorld.y;
			meas.at<float>(2) = posImage.x;
			meas.at<float>(3) = posImage.y;
			meas.at<float>(4) = ballRect[0].width;
			meas.at<float>(5) = ballRect[0].height;
			//cout << "meas: " << meas << endl;
			//cout << "measurement exists" << endl;
			if (reinitialize) { //! First detection!
				//cout << "lost" << endl;
				//! >>>> Initialization
				setIdentity(kFilter.errorCovPre);
				kFilter.measurementNoiseCov = measNoiseCov;
				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);
				state.at<float>(2) = 0;
				state.at<float>(3) = 0;
				state.at<float>(4) = 0;
				state.at<float>(5) = 0;
				state.at<float>(6) = meas.at<float>(2);
				state.at<float>(7) = meas.at<float>(3);
				state.at<float>(8) = 0;
				state.at<float>(9) = 0;
				state.at<float>(10) = meas.at<float>(4);
				state.at<float>(11) = meas.at<float>(5);
				//cout << "Staet: " << state << endl;
				//! <<<< Initialization
				kFilter.statePost = state;
				lost = false;
				losingBall = false;
				reinitialize = false;
				timeSinceLost = 0.f;
			} else {
				kFilter.measurementNoiseCov = measNoiseCov;
				//cout << "not lost so correcting" << endl;
				//cout << "Meas: " << meas << endl;
				//cout << "kFilter.measurementNoiseCov: " << kFilter.measurementNoiseCov << endl;
				kFilter.correct(meas); //! Kalman Correction
			}
		}
	}
	if (losingBall) {
		timeSinceLost = visionModule->getModuleTime() - timeAtLost;
	}
	if (timeSinceLost >= 2.f) lost = true;
	motionState = kFilter.statePost.colRange(0,1).rowRange(0,6);
	//cout << "timeSinceLost: " << timeSinceLost << endl;
}

void
BallTracker::setupKalmanFilter()
{
	unsigned int type = CV_32F;
	kFilter = KalmanFilter(stateSize, measSize, contrSize, type);
	state = Mat(stateSize, 1, type);
	meas = Mat(measSize, 1, type);
	measNoiseCov = Mat(measSize, measSize, type);
	infMeasNoiseCov = Mat(measSize, measSize, type);
	
	//! Variables other than kalman filter
	motionState = Mat(motionStateSize, 1, type); //! px, py, vx, vy, ax, ay
	motionStateTransition = Mat(motionStateSize, motionStateSize, type);
	
	//! Transition State Matrix A
	//! [ 1 0  dT   0  dT^2/2  0       0   0  0   0   0  0]
	//! [ 0 1   0  dT  0       dT^2/2  0   0  0   0   0  0]
	//! [ 0 0   1   0  dT      0       0   0  0   0   0  0]
	//! [ 0 0   0   1  0       dT      0   0  0   0   0  0]
	//! [ 0 0   0   0  1       0       0   0  0   0   0  0]
	//! [ 0 0   0   0  0       1       0   0  0   0   0  0]
	//! [ 0 0   0   0  0       0       1   0  dT  0   0  0]
	//! [ 0 0   0   0  0       0       0   1  0   dT  0  0]
	//! [ 0 0   0   0  0       0       0   0  1   0   0  0]
	//! [ 0 0   0   0  0       0       0   0  0   1   0  0]
	//! [ 0 0   0   0  0       0       0   0  0   0   1  0]
	//! [ 0 0   0   0  0       0       0   0  0   0   0  1]
	setIdentity(kFilter.transitionMatrix);

	kFilter.transitionMatrix.at<float>(2) = cycleTime;
	kFilter.transitionMatrix.at<float>(4) = cycleTime * cycleTime / 2;
	kFilter.transitionMatrix.at<float>(15) = cycleTime;
	kFilter.transitionMatrix.at<float>(17) = cycleTime * cycleTime / 2;
	kFilter.transitionMatrix.at<float>(28) = cycleTime;
	kFilter.transitionMatrix.at<float>(41) = cycleTime;
	kFilter.transitionMatrix.at<float>(80) = cycleTime;
	kFilter.transitionMatrix.at<float>(93) = cycleTime;

	//! Measure Matrix H
	//! [ 1 0 0 0 0 0 0 0 0 0 0 0 ] [  x     ]
	//! [ 0 1 0 0 0 0 0 0 0 0 0 0 ] [  y     ]
	//! [ 0 0 0 0 0 0 1 0 0 0 0 0 ] [  xdot  ]
	//! [ 0 0 0 0 0 0 0 1 0 0 0 0 ] [  ydot  ]
	//! [ 0 0 0 0 0 0 0 0 0 0 1 0 ] [  xddot ]
	//! [ 0 0 0 0 0 0 0 0 0 0 0 1 ] [  yddot ]
	//!                             [  pIx   ]
	//!                             [  pIy   ]
	//!                             [  vIx   ]
	//!                             [  vIy   ]
	//!                             [  w     ]
	//!                             [  h     ]
	kFilter.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kFilter.measurementMatrix.at<float>(0) = 1.0f;
	kFilter.measurementMatrix.at<float>(13) = 1.0f;
	kFilter.measurementMatrix.at<float>(30) = 1.0f;
	kFilter.measurementMatrix.at<float>(43) = 1.0f;
	kFilter.measurementMatrix.at<float>(58) = 1.0f;
	kFilter.measurementMatrix.at<float>(71) = 1.0f;

	//! Process Noise Covariance Matrix Q
	//! [ eX   0    0     0     0     0     0     0     0     0    0    0]
	//! [ 0    eY   0     0     0     0     0     0     0     0    0    0]
	//! [ 0    0    eVX   0     0     0     0     0     0     0    0    0]
	//! [ 0    0    0     eVY   0     0     0     0     0     0    0    0]
	//! [ 0    0    0     0     eAx   0     0     0     0     0    0    0]
	//! [ 0    0    0     0     0     eAY   0     0     0     0    0    0]
	//! [ 0    0    0     0     0     0     eIx   0     0     0    0    0]
	//! [ 0    0    0     0     0     0     0     eIy   0     0    0    0]
	//! [ 0    0    0     0     0     0     0     0     eVIx  0    0    0]
	//! [ 0    0    0     0     0     0     0     0     0     eVIy 0    0]
	//! [ 0    0    0     0     0     0     0     0     0     0    eW   0]
	//! [ 0    0    0     0     0     0     0     0     0     0    0   eH]
	kFilter.processNoiseCov.at<float>(0) = 0.001;
	kFilter.processNoiseCov.at<float>(13) = 0.001;
	kFilter.processNoiseCov.at<float>(26) = 0.025;
	kFilter.processNoiseCov.at<float>(39) = 0.025;
	kFilter.processNoiseCov.at<float>(52) = 0.001;
	kFilter.processNoiseCov.at<float>(65) = 0.001;
	kFilter.processNoiseCov.at<float>(78) = 0.01;
	kFilter.processNoiseCov.at<float>(91) = 0.01;
	kFilter.processNoiseCov.at<float>(104) = 5.0;
	kFilter.processNoiseCov.at<float>(117) = 5.0;
	kFilter.processNoiseCov.at<float>(130) = 0.01;
	kFilter.processNoiseCov.at<float>(143) = 0.01;

	setIdentity(measNoiseCov);
	setIdentity(infMeasNoiseCov);

	//! Measurement Noise Covariance Matrix Q
	//! [ eX   0    0     0     0    0]
	//! [ 0    eY   0     0     0    0]
	//! [ 0    0    eIx   0     0    0]
	//! [ 0    0    0     eIy   0    0]
	//! [ 0    0    0     0     eW   0]
	//! [ 0    0    0     0     0   eH]
	measNoiseCov.at<float>(0) = 1e-2;
	measNoiseCov.at<float>(7) = 1e-2;
	measNoiseCov.at<float>(14) = 1e-1;
	measNoiseCov.at<float>(21) = 1e-1;
	measNoiseCov.at<float>(28) = 1e-1;
	measNoiseCov.at<float>(35) = 1e-1;
	kFilter.measurementNoiseCov = measNoiseCov;

	infMeasNoiseCov.at<float>(0) = 1e6;
	infMeasNoiseCov.at<float>(7) = 1e6;
	infMeasNoiseCov.at<float>(14) = 1e6;
	infMeasNoiseCov.at<float>(21) = 1e6;
	infMeasNoiseCov.at<float>(28) = 1e6;
	infMeasNoiseCov.at<float>(35) = 1e6;
}

void
BallTracker::getBallPreview(
	vector<Mat>& ballPreview, 
	const unsigned& n, 
	const float& dt)
{
	setIdentity(motionStateTransition);
	float dtSquare = dt*dt/2.f;
	motionStateTransition.at<float>(2) = dt;
	motionStateTransition.at<float>(4) = dtSquare;
	motionStateTransition.at<float>(9) = dt;
	motionStateTransition.at<float>(11) = dtSquare;
	motionStateTransition.at<float>(16) = dt;
	motionStateTransition.at<float>(23) = dt;	
	ballPreview.resize(n);
	ballPreview[0] = motionState;
	for (size_t i = 1; i < ballPreview.size(); ++i) {
		ballPreview[i] = motionStateTransition * ballPreview[i-1];
	}
}
