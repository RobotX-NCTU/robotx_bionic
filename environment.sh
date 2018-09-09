#!/bin/bash
#[ -z "$DUCKIETOWN_ROOT" ] && { echo "Need to set DUCKIETOWN_ROOT - configuration is invalid (!)";  }
[ -z "$HOSTNAME"        ] && { echo "Need to set HOSTNAME.";        }

# Do not compile Lisp messages
# XXX: not sure if this is the place to put this.
export ROS_LANG_DISABLE=gennodejs:geneus:genlisp

shell=`basename $SHELL`
echo "Activating ROS with shell: $SHELL"
source /opt/ros/kinetic/setup.$shell

export HOSTNAME=$HOSTNAME
export ROS_HOSTNAME=$HOSTNAME.local
echo "Set ROS_HOSTNAME to: $ROS_HOSTNAME"

#export DUCKIETOWN_ROOT=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
#echo "Set DUCKIETOWN_ROOT to: $DUCKIETOWN_ROOT"

#export PYTHONPATH=$DUCKIETOWN_ROOT/catkin_ws/src:$PYTHONPATH
#echo "Set PYTHONPATH to: $PYTHONPATH"

# Cannot make machines before building
# echo "Building machines file..."
# make -C $DUCKIETOWN_ROOT machines

echo "Activating development environment..."
source ~/robotx_bionic/catkin_ws/devel/setup.$shell

if [ 2015 -ge $(date +%Y) ];          
then
    >&2 echo "Error! Time travel detected. System time is: $(date)"
fi

exec "$@" #Passes arguments. Need this for ROS remote launching to work.
