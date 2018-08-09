#!/bin/bash

trap killgroup SIGINT

killgroup() {
  echo killing...
  kill 0
}

$PATH_TO_VREP_DIR/vrep.sh $PATH_TO_TEAM_NUST_DIR/Utils/Simulator/Scene/KickTests.ttt -s
# & sleep 8s;
#$PATH_TO_TEAM_NUST_DIR/Utils/Simulator/build/VREP-Naoqi-Sim
