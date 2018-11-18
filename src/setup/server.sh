#!/bin/sh
#UE4_TRUE_SCRIPT_NAME=$(echo \"$0\" | xargs readlink -f)
#UE4_PROJECT_ROOT=$(dirname "$UE4_TRUE_SCRIPT_NAME")
UE4_PROJECT_ROOT=/home/chouer/workspace/carla/carla_0.8.2/
chmod +x "$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4"
#"$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4" CarlaUE4 -carla-settings="/home/chouer/workspace/expr2/ReMotionPlanning/src/setup/e2.ini"
#"$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4" CarlaUE4 -carla-server -benchmark -fps=200
noserver=0
a=$1
echo $a
if [ $a == $noserver ]
then
    echo "not server"
    "$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4" CarlaUE4 -QualityLevel=Low
else
    "$UE4_PROJECT_ROOT/CarlaUE4/Binaries/Linux/CarlaUE4" CarlaUE4 -carla-server -benchmark -fps=200 -QualityLevel=Low
fi
