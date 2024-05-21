# precision-landing-uav-ardu-roscam

Prerequisites-:  

    Ubuntu (18.04 LTS) Full 3D graphics hight recommended. 

    Gazebo version 8.x or greater (Recomended 9.6) 

    ROS melodic (Required to work with Gazebo) 

    MAVROS 

If have any problem follow the below link-:  

https://github.com/r0ch1n/ardupilot_gazebo_roscam 

 

source /opt/ros/melodic/setup.bash 

 

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH 

exportGAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH 

exportGAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH  

export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:$GAZEBO_PLUGIN_PATH 

 

 

Add these lines in your bashrc file.  

 

Open a Terminal and run these lines 

 

    source ~/ardupilot_gazebo_roscam/devel/setup.bash 

    roslaunch ardupilot_gazebo iris_with_roscam.launch 

 

In a second terminal run these lines 

 

    cd ~/ardupilot/ArduCopter 
 
    sim_vehicle.py -f gazebo-iris --console --map 

 

In third Terminal run these lines  

 

    source ~/ardupilot_gazebo_roscam/devel/setup.bash 

    Rosrun ardupilot_gazebo sitl.py 
