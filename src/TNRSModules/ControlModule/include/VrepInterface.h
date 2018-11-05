/**
 * @file Simulator/Interface.h
 *
 * This file declares the class Interface
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
#define VREP_SERVER_PORT 19999
#define ROBOT_MODEL "NAOH25V50"
#define ROBOT_PORT 9559
#define UPDATE_PERIOD_MS 10
#define CAMERA_UPDATE_PERIOD 0.1

extern "C" {
    #include "extApi.h"
}

/**
 * @class Interface
 * @brief Sets up the connection between the vrep cpp remote api server
 *   and the naoqi's simulated robot.
 */ 
class Interface
{
public:
  /**
   * Constructor
   */ 
	Interface();
  
  /**
   * Destructor
   */ 
	~Interface();
  
  /**
   * Initializes the interface
   */ 
	void init();
  
  /**
   * Updates the robot state in simulation and in nao sim
   */ 
	void update();
	
private:
  /**
   * Launches the naoqi simulated robot based on the given simulator
   * sdk and robot model along with naoqi-hal. Furthermore, it sets all
   * nao actuators to their zero position.
   */ 
	void setupNaoqi();
  
  /**
   * Sets up the connnection with vrep remote api servers running on the 
   * simulator
   */ 
	void setupVrep();
  
  /**
   * Sets up the connection between vrep sensor/actuator handles and 
   * naoqi simulated robot sensor/actuator handles
   */ 
  void setupInterface();
  
  /**
   * Updates all the sensors in naoqi from vrep
   */ 
	void updateNaoqi();
  
  /**
   * Updates all the actuator commands in vrep from naoqi
   */ 
	void updateVrep();
	
  /**
   * Updates the joint states from vrep to naoqi
   */ 
	void jointsUpdate();
  
  /**
   * Updates the inertial sensor states from vrep to naoqi
   */ 
	void imuUpdate();
  
  /**
   * Updates the camera sensor images from vrep to naoqi
   */ 
	void camUpdate();
  
  /**
   * Updates the foot sensor states from vrep to naoqi
   */ 
	void fsrUpdate();
	
  /**
   * Gets the robot joint handles from vrep
   */ 
	void getJointHandles();
  
  /**
   * Gets the camera sensor handles from vrep
   */ 
	void getCamHandles();
  
  /**
   * Gets the foot sensor handles from vrep
   */ 
	void getFsrHandles();
	
  //! Naoqi simulated robot launcher
  Sim::SimLauncher* naoqiSim;
  
  //! Naoqi simulator sdk path
  string naoqiSimPath;
  
  //! Pointer for naoqi based robot model
  Sim::Model* naoqiModel;
  
  //! Pointer for naoqi based robot hardware abstraction layer 
  Sim::HALInterface* naoqiHal;
  
  //! The nao model type used in the simulation
  string modelType;
  
  //! The port at which naoqi simulated robot is set up
  int naoqiPort;
  
  //! Period of update cycle for interface (extraction and update)
  int periodMinMS; 
  
  //! Client id for connection with vrep sensor streaming server
  int clientId;
  
  //! Vrep handles for robot joints
	vector<simxInt> jointHandles;
  
  //! Container for received joint position values
  vector<float> jointPositions;
  
  //! Vrep handles for robot cameras
	vector<simxInt> camHandles;
  
  //! Pointer to the recveived camera image
	simxUChar* image;
  
  //! Vrep handles for robot camera resolutions
	vector<simxInt*> vrepCamRes;
  
  //! Vrep handles for robot force sensors
	vector<simxInt> fsrHandles;
  
  //! Pointer to the recveived fsr sensor values
  vector<simxFloat*> fsrSensorVals;
    
  //! Vrep pointer to fsr sensor states
	vector<simxUChar> fsrStates;
  
  //! Naoqi definition of camera resolutions 
  //! see sdk/include/alvisiondefinitions.h for details
	vector<unsigned> naoqiCamRes;
  
  //! A vector of naoqi represented joint names 
  vector<string> jointNames;

  //! A vector of naoqi joint sensor handles
  vector<const Sim::AngleSensor*> jointSensors;
  
  //! A vector of naoqi joint actuator handles
  vector<const Sim::AngleActuator*> jointActuators;
  
  //! A vector of naoqi camera handles
  vector<const Sim::CameraSensor*> camSensors;
  
  //! A vector of naoqi fsr handles
  vector<const Sim::FSRSensor*> fsrSensors;
  
  //! A vector of naoqi inertial sensor handles
  const Sim::InertialSensor* inertialSensor;
  
  //! Last camera update time
  high_resolution_clock::time_point camLastUpdateTime;
};
