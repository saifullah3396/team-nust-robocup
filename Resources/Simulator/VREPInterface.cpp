/**
 * @file Simulator/VREPInterface.cpp
 *
 * This file implements the class that starts a remote Api client to connect
 * with the robot simulated in Vrep.
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#include "VREPInterface.h"

VREPInterface::VREPInterface() 
{
  if(naoqiSim) {
    delete naoqiSim;
    naoqiSim = NULL;
  }
  if(naoqiModel) {
    delete naoqiModel;
    naoqiModel = NULL;
  }
  if(naoqiHal) {
    delete naoqiHal;
    naoqiHal = NULL;
  }
  naoqiSim = new Sim::SimLauncher();
  naoqiSimPath = NAOQI_SIM_SDK;
  modelType = ROBOT_MODEL;
  naoqiPort = ROBOT_PORT;
  
	jointHandles.resize(JOINTS);
	jointPositions.resize(JOINTS);
	//imuHandles.resize(2);
	
	camHandles.resize(CAMERAS);
	camSensors.resize(CAMERAS);
	naoqiCamRes.resize(CAMERAS);
	vrepCamRes.resize(CAMERAS);
	vrepCamRes[0] = new simxInt[2];
	vrepCamRes[1] = new simxInt[2];
	
	fsrHandles.resize(FSR_SENSORS);
	fsrSensors.resize(FSR_SENSORS);
	fsrSensorVals.resize(FSR_SENSORS);
	fsrStates.resize(FSR_SENSORS);
	for (int i = 0; i < fsrSensorVals.size(); ++i)
		fsrSensorVals[i] = new simxFloat[3];
	camLast = high_resolution_clock::now();
}

VREPInterface::~VREPInterface() 
{
  if(naoqiSim) {
    delete naoqiSim;
    naoqiSim = NULL;
	}
  if(naoqiModel) {
    delete naoqiModel;
    naoqiModel = NULL;
	}
  if(naoqiHal) {
    delete naoqiHal;
    naoqiHal = NULL;
	}
	simxFinish(sensorsClientId);
	simxFinish(cameraClientId);
}

void VREPInterface::start()
{
	setupNaoqi();
	setupVrep();
	setupInterface();
	//simxSynchronousTrigger(sensorsClientId);
	//simxSynchronous(sensorsClientId, 1);
	simxStartSimulation(sensorsClientId, simx_opmode_blocking);
	simxStartSimulation(cameraClientId, simx_opmode_blocking);
}

void VREPInterface::update()
{
	auto tStart = high_resolution_clock::now();
	updateNaoqi();
	updateVrep();  
  auto lastIterationTimeMS =
    duration_cast<milliseconds>(high_resolution_clock::now() - tStart).count();
  if(lastIterationTimeMS < periodMinMS) {
    int waitTimeMS = periodMinMS - lastIterationTimeMS;
    usleep(waitTimeMS*1000);
  }
	//clock_t lastIterationTime = clock() - iterationStartTime;
  //float lastIterationTimeMS = float (lastIterationTime) / float(1000);
  //cout << lastIterationTimeMS << endl;
   // if(lastIterationTimeMS < periodMinMS) {
//		int waitTimeMS = periodMinMS - lastIterationTimeMS;
  //      usleep(lastIterationTimeMS*1000);
//	}
	//simxSynchronousTrigger(sensorsClientId);
	//simxInt pingTime;
	//simxGetPingTime(clientId, &pingTime);  
}

void VREPInterface::updateNaoqi() 
{
	jointsUpdate();
	imuUpdate();
	fsrUpdate();
  duration<double> timeSpan = high_resolution_clock::now() - camLast;
	if (timeSpan.count() > 0.03) {
		camUpdate();
		camLast = high_resolution_clock::now();
	}
}

void VREPInterface::updateVrep() 
{
  for(unsigned int i = 0; i < jointNames.size(); ++i)
  {
	const Sim::AngleActuator* actuator;
  string actuatorName;
  if (jointNames[i] == "RHipYawPitch")
    actuatorName = "LHipYawPitch";
  else 
    actuatorName = jointNames[i];
	actuator = naoqiModel->angleActuator(actuatorName);
    while(!actuator) {
		actuator = naoqiModel->angleActuator(actuatorName);
	}
    float jointTarget = naoqiHal->fetchAngleActuatorValue(actuator);
    if(jointTarget != jointTarget)
		jointTarget = actuator->startValue();
	 simxSetJointTargetPosition(sensorsClientId,jointHandles[i],jointTarget,simx_opmode_oneshot);
  }
}

void VREPInterface::jointsUpdate()
{
	for (int i=0; i<(JOINTS); i++) {
		if(simxGetJointPosition(sensorsClientId,jointHandles[i],&jointPositions[i],simx_opmode_buffer)==simx_return_ok)
			naoqiHal->sendAngleSensorValue(jointSensors[i], jointPositions[i]);
	}
}

void VREPInterface::imuUpdate()
{
	vector<float> imu;
	imu.resize(7);
	simxGetFloatSignal(sensorsClientId, "angleX", &imu[0], simx_opmode_oneshot);
	simxGetFloatSignal(sensorsClientId, "angleY", &imu[1], simx_opmode_oneshot);
	//simxGetFloatSignal(sensorsClientId, "angleZ", &imu[1], simx_opmode_oneshot));
	simxGetFloatSignal(sensorsClientId, "accelerometerX", &imu[2], simx_opmode_oneshot);
	simxGetFloatSignal(sensorsClientId, "accelerometerY", &imu[3], simx_opmode_oneshot);
	simxGetFloatSignal(sensorsClientId, "accelerometerZ", &imu[4], simx_opmode_oneshot);
	simxGetFloatSignal(sensorsClientId, "gyroX", &imu[5], simx_opmode_oneshot);
	simxGetFloatSignal(sensorsClientId, "gyroY", &imu[6], simx_opmode_oneshot);
	//simxGetFloatSignal(sensorsClientId, "gyroZ", &imu[7], simx_opmode_oneshot));
	naoqiHal->sendInertialSensorValues(inertialSensor, imu);
}

void VREPInterface::camUpdate()
{
	for (int i=0; i<(CAMERAS); i++) {
		if (simxGetVisionSensorImage(cameraClientId, camHandles[i], vrepCamRes[i], &image, 0, simx_opmode_buffer)==simx_return_ok) {
			naoqiHal->sendCameraSensorValue(this->camSensors[i], &image[0], (Sim::CameraResolution)naoqiCamRes[i], Sim::COL_SPACE_RGB);
		}
	}
}

void VREPInterface::fsrUpdate()
{
	for (int i=0; i<(FSR_SENSORS); i++) {
		if(simxReadForceSensor(sensorsClientId, fsrHandles[i], &fsrStates[i], fsrSensorVals[i], 0, simx_opmode_buffer)==simx_return_ok)
			naoqiHal->sendFSRSensorValue(fsrSensors[i], fsrSensorVals[i][2]/9.80665);
	}
	
}

void VREPInterface::setupNaoqi()
{
	try {	
		string naoqiModelPath = naoqiSimPath + 
								 "/share/alrobotmodel/models/" + 
								 modelType + 
								 ".xml";

		naoqiModel = new Sim::Model(naoqiModelPath);
		naoqiHal = new Sim::HALInterface(naoqiModel, naoqiPort);
		if(!naoqiSim->launch(naoqiModel, naoqiPort, naoqiSimPath))
		{
			cout << "Could not launch naoqi sim." << endl;
		}
	} catch (exception &e){
		cout << e.what() << endl;
	}
	jointActuators = naoqiModel->angleActuators();
	jointSensors = naoqiModel->angleSensors();
	for(vector<const Sim::AngleActuator*>::const_iterator it =
			jointActuators.begin(); it != jointActuators.end(); ++it)
	{
		float actuatorPosition = (*it)->startValue();
		const Sim::AngleSensor* sensor = naoqiModel->angleSensor((*it)->name());
		naoqiHal->sendAngleSensorValue(sensor, actuatorPosition);
		jointNames.push_back((*it)->name());
	}
}

void VREPInterface::setupVrep()
{
	try {	
		simxFinish(-1);
		sensorsClientId = simxStart((simxChar*)VREP_SERVER_IP,
							   VREP_SERVER_PORT_SENSORS,true,true,2000,5);
		cameraClientId = simxStart((simxChar*)VREP_SERVER_IP,
							   VREP_SERVER_PORT_CAMERA,true,true,2000,5);
		if (sensorsClientId!=-1) {
			cout << "Connected to vrep remote API sensors server." << endl;
		} else {
			cout << "Connection to vrep remote API sensors server failed." << endl;
		}
		
		if (cameraClientId!=-1) {
			cout << "Connected to vrep remote API camera server." << endl;
		} else {
			cout << "Connection to vrep remote API camera server failed." << endl;
		}
	} catch (exception &e){
		cout << e.what() << endl;
	}
}

void VREPInterface::setupInterface()
{
	try {	
		getJointHandles();
		//getImuHandles();
		getFsrHandles();
		getCamHandles();
		
		vector<const Sim::CameraSensor*> camSensorPtrs = naoqiModel->cameraSensors();
		while(naoqiModel->cameraSensors().size() == 0) 
			camSensorPtrs = naoqiModel->cameraSensors();
		if (camSensorPtrs.size() >= 2)
		{
			for (int i = 0; i < camSensorPtrs.size(); ++i)
			{				
				if(camSensorPtrs[i]->name() == "CameraTop") {					
					this->camSensors[0] = camSensorPtrs[i];
					this->naoqiCamRes[0] = 3;//naoqiHal->cameraResolution(camSensorPtrs[i]);
					cout << "CameraTop"<< naoqiCamRes[0] << endl;
				}
				else if(camSensorPtrs[i]->name() == "CameraBottom") {
				  this->camSensors[1] = camSensorPtrs[i];
				  this->naoqiCamRes[1] = 2;//naoqiHal->cameraResolution(camSensorPtrs[i]);
				  cout << "CamerBot"<< naoqiCamRes[1] << endl;
				}
			}
			for (int i = 0; i < CAMERAS; ++i) {
				vector<simxInt> res(2);				
				switch(naoqiCamRes[i]) {
					case Sim::RES_80_60:
						res[0] = 80;
						res[1] = 60;
						break;
					case Sim::RES_160_120:
						res[0] = 160;
						res[1] = 120;
						break;
					case Sim::RES_320_240:
						res[0] = 320;
						res[1] = 240;
						break;
					case Sim::RES_640_480:
						res[0] = 640;
						res[1] = 480;
						break;
					case Sim::RES_1280_960:
						res[0] = 1280;
						res[1] = 960;
						break;
					case Sim::RES_UNKNOWN:
						if (i == TOP_CAM) {
							res[0] = 640;
							res[1] = 480;
						}
						else {
							res[0] = 320;
							res[1] = 240;
						}
						break;
				}
				if(i == TOP_CAM) {
					res[0] = 640;
					res[1] = 480;
				} else if (i == BOTTOM_CAM) {
					res[0] = 320;
					res[1] = 240;
				}
				simxSetObjectIntParameter(
					cameraClientId, 
					camHandles[i], 
					sim_visionintparam_resolution_x, 
					res[0], 
					simx_opmode_blocking
				);
				simxSetObjectIntParameter(
					cameraClientId, 
					camHandles[i], 
					sim_visionintparam_resolution_y, 
					res[1], 
					simx_opmode_blocking
				);
				simxGetVisionSensorImage(cameraClientId, camHandles[i], vrepCamRes[i], &image, 0, simx_opmode_streaming);
			}
			camUpdate();
		}
		else
			cout << "Could not find vision sensors handles in naoqi." << endl;
		 
		vector<const Sim::InertialSensor*> inertialSensors = naoqiModel->inertialSensors();
		if(inertialSensors.size() >= 1)
		{
			inertialSensor = inertialSensors[0];
			imuUpdate();
		}
		
		vector<const Sim::FSRSensor*> fsrSensors = naoqiModel->fsrSensors();
		if (fsrSensors.size() >= 8) {
			for(int i=0;i<fsrSensors.size();i++) {
				string name = fsrSensors[i]->name();
				if(name == "LFoot/FSR/FrontLeft")
						this->fsrSensors[0] = fsrSensors[i];
				else if(name == "LFoot/FSR/FrontRight")
						this->fsrSensors[1] = fsrSensors[i];   
				else if(name == "LFoot/FSR/RearLeft")
						this->fsrSensors[2] = fsrSensors[i];
				else if(name == "LFoot/FSR/RearRight")
						this->fsrSensors[3] = fsrSensors[i];
				else if(name == "RFoot/FSR/FrontLeft")
						this->fsrSensors[4] = fsrSensors[i];
				else if(name == "RFoot/FSR/FrontRight")
						this->fsrSensors[5] = fsrSensors[i];
				else if(name == "RFoot/FSR/RearLeft")
						this->fsrSensors[6] = fsrSensors[i];
				else if(name == "RFoot/FSR/RearRight")
						this->fsrSensors[7] = fsrSensors[i];
			}
			fsrUpdate();
		}
	} catch (exception &e) {
		cout << e.what() << endl;
	}
}

void VREPInterface::getJointHandles()
{
    simxGetObjectHandle(sensorsClientId,"HeadYaw#0", &jointHandles[HEAD_YAW_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"HeadPitch#0", &jointHandles[HEAD_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LShoulderPitch3#0", &jointHandles[L_SHOULDER_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LShoulderRoll3#0", &jointHandles[L_SHOULDER_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LElbowYaw3#0", &jointHandles[L_ELBOW_YAW_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LElbowRoll3#0", &jointHandles[L_ELBOW_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LWristYaw3#0", &jointHandles[L_WRIST_YAW_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RShoulderPitch3#0", &jointHandles[R_SHOULDER_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RShoulderRoll3#0", &jointHandles[R_SHOULDER_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RElbowYaw3#0", &jointHandles[R_ELBOW_YAW_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RElbowRoll3#0", &jointHandles[R_ELBOW_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RWristYaw3#0", &jointHandles[R_WRIST_YAW_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LHipYawPitch3#0", &jointHandles[L_HIP_YAW_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LHipRoll3#0", &jointHandles[L_HIP_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LHipPitch3#0", &jointHandles[L_HIP_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LKneePitch3#0", &jointHandles[L_KNEE_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LAnklePitch3#0", &jointHandles[L_ANKLE_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"LAnkleRoll3#0", &jointHandles[L_ANKLE_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RHipYawPitch3#0", &jointHandles[R_HIP_YAW_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RHipRoll3#0", &jointHandles[R_HIP_ROLL_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RHipPitch3#0", &jointHandles[R_HIP_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RKneePitch3#0", &jointHandles[R_KNEE_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RAnklePitch3#0", &jointHandles[R_ANKLE_PITCH_POSITION], simx_opmode_blocking);
    simxGetObjectHandle(sensorsClientId,"RAnkleRoll3#0", &jointHandles[R_ANKLE_ROLL_POSITION], simx_opmode_blocking);
    for (int i = 0; i < JOINTS; i++)
			simxGetJointPosition(sensorsClientId, jointHandles[i], &jointPositions[i], simx_opmode_streaming);
}

void VREPInterface::getCamHandles()
{
	simxGetObjectHandle(cameraClientId,"NAO_vision1#0", &camHandles[TOP_CAM], simx_opmode_blocking);
	simxGetObjectHandle(cameraClientId,"NAO_vision2#0", &camHandles[BOTTOM_CAM], simx_opmode_blocking);
}

void VREPInterface::getFsrHandles()
{
	simxGetObjectHandle(sensorsClientId,"NAO_LFsrFL#0", &fsrHandles[L_FOOT_FSR_FL], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_LFsrFR#0", &fsrHandles[L_FOOT_FSR_FR], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_LFsrRL#0", &fsrHandles[L_FOOT_FSR_RL], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_LFsrRR#0", &fsrHandles[L_FOOT_FSR_RR], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_RFsrFL#0", &fsrHandles[R_FOOT_FSR_FL], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_RFsrFR#0", &fsrHandles[R_FOOT_FSR_FR], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_RFsrRL#0", &fsrHandles[R_FOOT_FSR_RL], simx_opmode_blocking);
	simxGetObjectHandle(sensorsClientId,"NAO_RFsrRR#0", &fsrHandles[R_FOOT_FSR_RR], simx_opmode_blocking);
	for (int i = 0; i < (FSR_SENSORS); i++)
		simxReadForceSensor(sensorsClientId, fsrHandles[i], &fsrStates[i], fsrSensorVals[i], 0, simx_opmode_streaming);
}

/*void VREPInterface::getImuHandles() //!FIXME: VREP provides gyro and 
 * imu in the form of scripts using other sensors and not as separate entities.
 */
/*{
  simxGetObjectHandle(sensorsClientId,"Accelerometer_forceSensor#", &imuHandles[0], simx_opmode_blocking);
  simxGetObjectHandle(sensorsClientId,"GyroSensor#", &imuHandles[1], simx_opmode_blocking);
}*/
