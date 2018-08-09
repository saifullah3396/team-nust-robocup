/**
 * @file Simulator/VREPInterface.h
 *
 * This file declares a class that starts a remote Api client to connect
 * with the robot simulated in Vrep.
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 21 June 2017  
 */

#include <alnaosim/alnaosim.h>
#include <alnaosim/alnaosim_camera_definitions.h>
#include <alrobotmodel/alrobotmodel.h>
#include <alsimutils/sim_launcher.h>
#include <alproxies/almemoryproxy.h>

#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <unistd.h>

#include "SensorsEnum.h"

using namespace std;
using namespace std::chrono;

#define VREP_SERVER_IP "127.0.0.1"
#define VREP_SERVER_PORT_SENSORS 19999
#define VREP_SERVER_PORT_CAMERA 18999
#define ROBOT_MODEL "NAOH25V40"
#define ROBOT_PORT 9559

extern "C" {
    #include "extApi.h"
}

class VREPInterface
{
public:
	VREPInterface();
	~VREPInterface();
	void start();
	void update();
	
private:
	clock_t iterationStartTime;
	void setupNaoqi();
	void setupVrep();
	void updateNaoqi();
	void updateVrep();
	void setupInterface();
	
	void jointsUpdate();
	void imuUpdate();
	void camUpdate();
	void fsrUpdate();
	
	void getJointHandles();
	//void getImuHandles();
	void getCamHandles();
	void getFsrHandles();
	
	int sensorsClientId;
	int cameraClientId;
	vector<simxInt> jointHandles;
	vector<simxInt> camHandles;
	vector<simxInt> fsrHandles;
	vector<simxInt*> vrepCamRes;
	vector<unsigned> naoqiCamRes;
	simxUChar* image;
	//vector<simxInt> imuHandles;
	vector<float> jointPositions;
	vector<simxFloat*> fsrSensorVals;
	vector<simxUChar> fsrStates;
	
  Sim::SimLauncher* naoqiSim;
  Sim::Model* naoqiModel;
  Sim::HALInterface* naoqiHal;
	string naoqiSimPath, modelType;
	string robotNamespace;
	int naoqiPort;
  
  vector<string> jointNames;
  vector<const Sim::AngleSensor*> jointSensors;
  vector<const Sim::AngleActuator*> jointActuators;
  vector<const Sim::CameraSensor*> camSensors;
  vector<const Sim::FSRSensor*> fsrSensors;
  int simRes;
  const Sim::InertialSensor* inertialSensor;
  //!const Sim::SonarSensor* leftSonar; 
  //!FIXME: Add Sonar support later. Not on my agenda.
  //!const Sim::SonarSensor* rightSonar; 
  //!FIXME: Add Sonar support later. Not on my agenda.
  const AL::ALMemoryProxy* memoryProxy;
  static const int periodMinMS = 10;
  high_resolution_clock::time_point camLast;
};
