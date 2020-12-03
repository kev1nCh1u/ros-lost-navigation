# lost navigation

## start
    roscore
    
    rosrun map_server map_server /home/user/ros/lost_navigation/src/ros_map/test_0.yaml
    rosrun amcl amcl
    rosrun odometry odometry

    cd ~/ros/hokuyo_ws/
    rosrun hokuyo_node hokuyo_node

    rosrun rviz rviz -d /home/user/ros/lost_navigation/src/20201203.rviz


## Error: Device /dev/ttyACM0 is already locked
    lsof | grep /dev/ttyACM0
    kill -9 pocess_id