# TeamNUST RoboCup SPL
=======================================================

* Team-NUST   was   established   formally   in   2013   with   the   aim   of   carrying   out   research   in   the   rapidly  
progressing   field   of   humanoid   robotics,   artificial   intelligence,   machine   vision,   motion   planning,  
kinematics   and   navigation;   with   the   motivation   to   participate   in   RoboCup   Standard   Platform   League  
(SPL).   
* We   are   working   on   robust   and   predictable   kicking   motion,   multi   objective   behavior   coordination,  
motion   planning,   situational   awareness   based   on   efficient   perception   and   robust   probabilistic  
multiagent   localization. 

* This respository contains the source code of our software architecture designed for Aldebaran NAO robots.

## Getting Started

* For CMake configuration, set the environment variables as following:
```
echo 'export PATH_TO_TEAM_NUST_DIR=/path/to/team-nust-robocup-spl' >> ~/.bashrc 
echo 'export PATH_TO_SIM_DIR=/path/to/simulator-sdk' >> ~/.bashrc 
echo 'export PATH_TO_VREP_DIR=/path/to/vrep' >> ~/.bashrc 
```

### Prerequisites

* QiBuild. For installation of qibuild use:
```
sudo apt-get install python-pip
sudo pip install qibuild
```
* Updated version of naoqi-sdk, naoqi-simulator-sdk, and naoqi-cross-toolchain for code compilation. The respective sdks are updated according to our code dependencies and are only available to the team members from a private repository. For details of the updates, email <A href="mailto:saifullah3396@gmail.com">here</A>.
* V-REP_PRO_EDU_V3_4_0_Linux which can be downloaded from: http://www.coppeliarobotics.com/previousversions.html

* For local code compilation, the following script can be used to solve the code dependencies.
```
sudo apt-get install libfftw3-dev libasound2-dev libnlopt-dev liblapack-dev
```

### Installing
* For remote code execution, compile the code with naoqi-sdk toolchain following the given steps:
```
qitoolchain create remote /path/to/naoqi-sdk/toolchain.xml 
qibuild add-config remote -t remote
```
* Go to source folder team-nust-robocup-spl.
```
qibuild init
qibuild configure -c remote -DMODULE_IS_REMOTE=ON
qibuild make -c remote -DMODULE_IS_REMOTE=ON
```

* For cross compilation to run the code on robot code execution, compile the code with naoqi-sdk toolchain following the given steps:
```
qitoolchain create cross /path/to/naoqi-cross-toolchain/toolchain.xml 
qibuild add-config cross -t cross
```
* Go to source folder PATH_TO_TEAM_NUST_DIR
```
qibuild init
qibuild configure -c remote -DMODULE_IS_REMOTE=OFF
qibuild make -c remote -DMODULE_IS_REMOTE=OFF
```

## Deployment

* For execution and testing of the code, you can use naoqi-simulator-sdk to deploy a naoqi-sim.
* Furthermore, you can use choregraphe to connect to this simulator.

### Simulator Startup

For running the code, a simulated naoqi robot must be running. For that use:
```
cd PATH_TO_TEAM_NUST_DIR/Utils/Simulator
./simulated-nao.sh
```
* This successfully loads the naoqi-sim for code exeuction.

For dynamic simulations in vrep, follow the given steps:
```
cd PATH_TO_TEAM_NUST_DIR/Utils/Simulator
./vrep-simulation.sh
$PATH_TO_TEAM_NUST_DIR/Utils/Simulator/build/VREP-Naoqi-Sim
$PATH_TO_TEAM_NUST_DIR/build-tc/sdk/bin/simulator-startup-stiffness
```
* Run the main code:
```
./PATH_TO_TEAM_NUST_DIR/build-remote/sdk/bin/team-nust-robocup-spl
```

## Built With
* [NAOQI](http://doc.aldebaran.com/2-1) - The Naoqi documentation

## Authors
* <A href="mailto:saifullah3396@gmail.com">Saifullah</A>

## License
BSD

## Acknowledgments
We acknowledge the code usage of RoboCup SPL Team-BHuman, Team-Austrain Kangaroos, and Team-HTWK. Needs update for a detailed description.
