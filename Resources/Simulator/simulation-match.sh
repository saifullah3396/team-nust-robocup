#!/bin/bash

trap killgroup SIGINT

killgroup() {
  echo killing...
  kill 0
}

$PATH_TO_VREP_DIR/vrep.sh $PATH_TO_TEAM_NUST_DIR/Utils/Simulator/Scene/MatchRobocup.ttt -s
