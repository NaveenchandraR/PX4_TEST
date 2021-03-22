# PX4_TEST
Swarm test repo

Prerequisits: Clone PX4 Firmware repo to home

Also, catkin_ws has to be present with MAVROS installed in it.
I have used the following script to install it and build: (https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_ros_melodic.sh)


cd ~/Firmware

    source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default

    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo

    roslaunch px4 mavros_posix_sitl.launch

With this, Gazebo SITL should be running

Open 2 new tabs and run the following command in both tabs

    export HOST=uav_(id) // id = 0, 1, 2...

Tab 1:

    rosrun px4_test state

Here, a new topic is being generated, "/uav_(id)/combined" // id = 0, 1, 2...
This topic has to be communicated to the other UAV

Tab 2:

    rosrun px4_test offb

Here other UAVs are copying the leader UAVs position and maintaining formation. As for the leader, we need to call a service to change its position.

In the leader UAV, open a new tab to make these service calls and change position.

    rosservice call /uav_0/leader_pose "pos_x: 5.0
    pos_y: 5.0
    pos_z: 5.0" 

    // Here 0 is taken as leader. But we can change it to any UAV that we want

Initially lets call the service using the terminal inside the leader to change position.
If this is working properly, we will use another system and use it as a GCS and call the service from that system
    ""/uav_(id)/leader_pos" // id = leader id"