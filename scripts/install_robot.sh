#!/bin/sh

BUILD="Release/Debug"
ROBOTS="Nu-11/Nu-12/Nu-13/Nu-14/Nu-15"
ROBOT=""
IP_PREFIX="192.168.30"

usage()
{
    echo "Usage:"
    echo "./copy_config.sh -b=<BUILD> -t=<TOOLCHAIN> -r=<ROBOT>"
    echo ""
    echo " -h | --help : Displays the help"
    echo " -b | --build : Build type (" $BUILD ")"
    echo " -t | --tool-chain : Toolchain name used in code compilation (" $TOOLCHAIN ")"
    echo " -r | --robot : Name of the robot for which calibration is needed (" $ROBOTS ")"
    echo ""
}

while [ "$1" != "" ]; do
    PARAM=`echo $1 | awk -F= '{print $1}'`
    VALUE=`echo $1 | awk -F= '{print $2}'`
    case $PARAM in
        -h | --help)
            usage
            exit
            ;;
        -t  | --tool-chain )
            TOOLCHAIN=$VALUE
            ;;
        -r | --robot)
            ROBOT=$VALUE
            ;;
        -b  | --tool-chain )
            BUILD=$VALUE
            ;;
        *)
            echo "ERROR: unknown parameter \"$PARAM\""
            usage
            exit 1
            ;;
    esac
    shift
done

if [ "$ROBOT" != "Nu-11" ] && 
   [ "$ROBOT" != "Nu-12" ] && 
   [ "$ROBOT" != "Nu-13" ] && 
   [ "$ROBOT" != "Nu-14" ] && 
   [ "$ROBOT" != "Nu-15" ]; then
    echo "Invalid robot name: $ROBOT . Options are: ( $ROBOTS )."
    exit 1;
fi

if [ "$BUILD" = "" ]; then
  echo "Please provide the build type to continue."
  exit 1
fi

ROBOT_NUM=""
if [ "$ROBOT" = "Nu-11" ]; then
  ROBOT_NUM=1
elif [ "$ROBOT" = "Nu-12" ]; then
  ROBOT_NUM=2
elif [ "$ROBOT" = "Nu-13" ]; then
  ROBOT_NUM=3
elif [ "$ROBOT" = "Nu-14" ]; then
  ROBOT_NUM=4
elif [ "$ROBOT" = "Nu-15" ]; then
  ROBOT_NUM=5
fi

ROBOT_DIR=$PATH_TO_TEAM_NUST_DIR/config/Robots/Nu-1$ROBOT_NUM
CROSS_DEPENDS_DIR=$PATH_TO_TEAM_NUST_DIR/cross-depends
BUILD_DIR=$PATH_TO_TEAM_NUST_DIR/build/$BUILD/$TOOLCHAIN/lib
rsync -r $ROBOT_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/config
rsync -r $CROSS_DEPENDS_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends
rsync -r $BUILD_DIR/* nao@$IP_PREFIX.$ROBOT_NUM:/home/nao/depends/lib
