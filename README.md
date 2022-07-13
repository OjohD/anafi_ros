# Anafi 4K Drone with ROS
This repository contains step-by-step instructions for configuring the Olympe SDK for use in ROS. Experiments are tested first in the Parrot Sphinx Simulator. Then later on the actual Anafi 4K drone. 

Tested on  Lenovo Y5 i7 10th gen RTX 2060 laptop running ROS Noetic on Ubuntu 20.04. 

## Setting up your ROS workspace
clone this repository to your src folder as follows.

    mkdir -p ~/anafi_ws/src
    cd ~/anafi_ws/src

then build and source your workspace
    
    catkin_make
    source devel/setup.bash

#
# Installing Olympe (Parrot Ground SDK)
    mkdir -p ~/code/parrot-groundsdk
    cd ~/code/parrot-groundsdk
    sudo snap install git-repo
    git config --global user.email "you@example.com"
    git config --global user.name "Your Name"
    repo init -u https://github.com/Parrot-Developers/groundsdk-manifest.git
    repo sync
    ./products/olympe/linux/env/postinst
    ./build.sh -p olympe-linux -A all final -j


## Solution to Olympe Installation Errors
This [link](https://github.com/andriyukr/olympe_bridge) fixes all issues you may face.  


## Getting Olympe to Work With ROS
Olympe installs to a virtual environment. Thus it is not readily available to be used in the base environment in linux. Therefore to get it to work with ROS, the ROS environment must be sourced into the Olympe virtual environment. This can be done by creating a custom bash script with the following contents:

    #!/bin/bash


    [[ $0 != $BASH_SOURCE ]] && \
        SCRIPT_PATH=$(realpath $BASH_SOURCE) || \
        SCRIPT_PATH="`readlink -f "$0"`"
    GSDK_DIR="`dirname "$SCRIPT_PATH"`"

    # The following line installs Olympe Python dependencies into your Python
    # virtual environement. It should only be necessary the first time and can
    # be commented out after that.
    pip3 install -r "$GSDK_DIR/packages/olympe/requirements.txt"

    # Add Olympe and GSDK Python dependencies to your PYTHONPATH
    export PYTHONPATH="${PYTHONPATH}:/$GSDK_DIR/out/olympe-linux/final/usr/lib/python/site-packages"

    # Add Olympe GSDK C dependencies to LD_LIBRARY_PATH
    source "$GSDK_DIR/out/olympe-linux/final/native-wrapper.sh"

    # Add ROS and your catkin workspace
    source /opt/ros/noetic/setup.bash
    source ~/anafi_ws/devel/setup.bash


save the contents above to a bash script eg. olympe_ros_env.sh. Replace *<anafi_ws>* with the name of your catkin workspace.  

    cd ~/code/parrot-groundsdk
    gedit olympe_ros_env.sh

### Running the script
Everytime you need to work with Olympe, source the custom environment in the active terminal. Followed by any of your ros-based scripts or launch files. 

    source ~/code/parrot-groundsdk/olympe_ros_env.sh

**NB.** Every new terminal should be sourced with the olympe_ros_env.sh

#

# Installing Parrot Sphinx Simulator 
Follow the instructions from [Parrot Developers](https://developer.parrot.com/docs/sphinx/installation.html) to install the latest version of the simulator on your Ubuntu 20.04 system. 

You will need to contact the developers for access to their software. 

Once the simulator is installed, you must restart your PC before you can start the simulator for the first time. 


## Running the Simulator
Before the simulator can be started, a service called 'firmwared' must be started. In your terminal do the following:

    sudo systemctl start firmwared.service

To check if the service started:

    fdc ping

You should get a respond with "PONG"


Start the simulator 

    sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi4k.drone"::firmware="ftp://<user>:<password>@ftp2.parrot.biz/versions/anafi/pc/%23latest/images/anafi-pc.ext2.zip

In a new terminal, launch your UE4 application.

    parrot-ue4-empty

### Fix for WIFI Disappearing
To prevent your wifi interface from being stolen by the simulator edit the .drone file you're working with

    sudo gedit /opt/parrot-sphinx/usr/share/sphinx/drones/anafi.drone

replace the line:
> <wifi_iface>auto</wifi_iface> 

with 

> <wifi_iface></wifi_iface>

Save the file and relaunch sphinx simulator.


## Testing Olympe with Parrot Sphinx Simulator

    source ~/code/parrot-groundsdk/olympe_ros_env.sh
    rosrun anafi_examples takeoff.py

#
# References
* [Andriy Sarabakha](https://github.com/andriyukr/olympe_bridge)
* [Carl Hildebrant](https://github.com/hildebrandt-carl/MixedRealityTesting)