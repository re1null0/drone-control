# Autonomous Drone Positioning and Real-time Object Detection

    
    1. Single-shot Deep Learning based Computer Vision Algorithm for the detection of occupancy of the car parking spaces that are visible to the camera mounted on the drone in real time.
    
    2. Custom control scripts for autonomously controlling the drone.
    
    3. Platform independent web page for autonomously controlling the drone. 

# Run the website locally

## Run frontend

Open a terminal and cd into the drone-frontend folder

`cd drone-frontend`

Start the server

`npm start`

You can see the site live at http://localhost:4200/

## Run Backend 

Open another terminal and cd into the `drone_backend` folder 

`cd drone_backend`

Run the shell script to start the backend

`./run.sh`

## Run Fire Simulation

Open another terminala and cd into `fireRL`

`cd fireRL`

Run this command to start the server:

`uvicorn main:app --reload --host 0.0.0.0 --port 8000`

## Run Camera Feed

Open another terminal and cd into `drone_backend`

`cd drone_backend`

Run the camera script:

`python camera_feed.py`


    
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

# How to Run the Docker Environment - to access ROS1

## Intermediary computer (1):
```
. drone_senior_design/devel/setup.bash
roslaunch hawk_tracking vicon_hawk.launch
```

## Intermediary computer (2):
```
. drone_senior_design/devel/setup.bash
rosrun hawk_tracking odom_relay.py
```

## Jetson:
```
ssh riya@100.77.20.58
sudo docker run -it --network=host --rm --runtime nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v /home/riya/drone-control:/workspace --device=/dev/ttyACM0:/dev/ttyACM0 ros_noetic_focal
``` 

In the Environment (1):
```
. /workspace/vicon_setup/devel/setup.bash
roscore & rosrun mocap_vicon odom_receiver.py
```

In the Environment (2):
```
. /workspace/vicon_setup/devel/setup.bash
[run scrips for control]
```