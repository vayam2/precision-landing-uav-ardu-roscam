# precision-landing-uav-ardu-roscam

Prerequisites-:  

    Ubuntu (18.04 LTS) Full 3D graphics hight recommended. 

    Gazebo version 8.x or greater (Recomended 9.6) 

    ROS melodic (Required to work with Gazebo) 

    MAVROS 

Install ardupilot_gazebo_roscam from the following the link-:  

https://github.com/r0ch1n/ardupilot_gazebo_roscam 



Once the setup from the above link is complete add the my_code folder in the src/ardupilot_gazebo folder and rebuild the ROS package ardupilot_gazebo_roscam



add the given below lines to your bash.rc if not already




source /opt/ros/melodic/setup.bash 

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH 

exportGAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH 

exportGAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH  

export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:$GAZEBO_PLUGIN_PATH 

 

  

Open a Terminal and run these lines 

 

    source ~/ardupilot_gazebo_roscam/devel/setup.bash 

    roslaunch ardupilot_gazebo iris_with_roscam.launch 

 

In a second terminal run these lines 

 

    cd ~/ardupilot/ArduCopter 
 
    sim_vehicle.py -f gazebo-iris --console --map 

 

In third Terminal run these lines  

 

    source ~/ardupilot_gazebo_roscam/devel/setup.bash 

    Rosrun ardupilot_gazebo sitl.py 
