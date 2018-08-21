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
qitoolchain create <REMOTE_TOOLCHAIN> /path/to/naoqi-sdk/toolchain.xml 
qibuild add-config <REMOTE_TOOLCHAIN> -t remote
```
* Go to source folder team-nust-robocup-spl.
```
qibuild init
make configure REMOTE=<REMOTE_TOOLCHAIN>
make install REMOTE=<REMOTE_TOOLCHAIN>

```

* For cross compilation to run the code on robot code execution, compile the code with naoqi-sdk toolchain following the given steps:
```
qitoolchain create <CROSS_TOOLCHAIN> /path/to/naoqi-cross-toolchain/toolchain.xml 
qibuild add-config <CROSS_TOOLCHAIN> -t cross
```
* Go to source folder PATH_TO_TEAM_NUST_DIR
```
qibuild init
make configure CROSS=<CROSS_TOOLCHAIN>
make install CROSS=<CROSS_TOOLCHAIN>
```

* For more make parameters, see Makefile

## Deployment

* For execution and testing of the code, you can use naoqi-simulator-sdk to deploy a naoqi-sim.
* Furthermore, you can use choregraphe to connect to this simulator.

### Remote usage on real robot

For running the code on a real robot through remote connection, 
connect to the robot via ethernet-lan/wifi and use:
```
./PATH_TO_TEAM_NUST_DIR/build/<BUILD_TYPE>/remote/bin/tnrs-module --pip <ROBOT_IP> --pport <ROBOT_PORT>
```
Where the <BUILD_TYPE>=Debug/Release depending on the build configuration.

### Local usage on real robot

For running the code on a real robot through cross build, use the 
following scripts to copy all the necessary files (libraries and cfg files)
on the robot.
```
./scripts/copyCfg <ROBOT_NUMBER>
./scripts/copyFiles <ROBOT_NUMBER>
```
The robot number is defined from [1-5] and defines the ip as 192.168.30.<ROBOT_NUMBER>.
This number is also used to copy the configuration files from the given robot's dir.
A robot directory is given as Config/Robots/Nu-1<ROBOT_NUMBER>.

Once the files are copied on the robot, use the following script to configure
robot to run the code on the robot.
```
nao$RobotName:~$ nao stop
nao$RobotName:~$ bin/toteamnust 
nao$RobotName:~$ naoqi
```

### Remote usage on the simulator
To setup the simulations, see the documentation for vrep-naoqi-simulations.
Once the simulator is up and running, use the following commands to run the code on
simulated robot:
```
./PATH_TO_TEAM_NUST_DIR/build/<BUILD_TYPE>/remote/bin/tnrs-module --pip <ROBOT_IP> --pport <ROBOT_PORT>
```
Where the <BUILD_TYPE>=Debug/Release depending on the build configuration.

## Built With
* [NAOQI](http://doc.aldebaran.com/2-1) - The Naoqi documentation

## Authors
* <A href="mailto:saifullah3396@gmail.com">Saifullah</A>

## License
BSD

## Acknowledgments
We acknowledge the code usage of RoboCup SPL Team-BHuman, Team-Austrain Kangaroos, and Team-HTWK. Needs update for a detailed description.
