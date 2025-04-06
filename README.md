# Autonomous Drone Positioning and Real-time Object Detection

    
    1. Single-shot Deep Learning based Computer Vision Algorithm for the detection of occupancy of the car parking spaces that are visible to the camera mounted on the drone in real time.
    
    2. Custom control scripts for autonomously controlling the drone.
    
    3. Platform independent web page for autonomously controlling the drone. 
    
# How to Run SITL Gazebo Simulation

In one terminal, open the ardupilot folder

`cd ardupilot/ArduCopter`

Run this command to start SITL:

 `../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console`

Then, in another terminal run Gazebo:

`cd gz_ws/src/build`

Run these commands to configure the environment:

`export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH`

Finally, run Gazebo

`gz sim -v4 -r iris_runway.sdf`