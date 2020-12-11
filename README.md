# lost navigation

## start
    roscore
    
    rosrun map_server map_server /home/user/ros/lost_navigation/src/ros_map/test_0.yaml
    rosrun amcl amcl
    rosrun odometry odometry
    rosrun hokuyo_node hokuyo_node

    rosrun rviz rviz -d /home/user/ros/lost_navigation/src/20201203.rviz

## debug
    rosrun rqt_tf_tree rqt_tf_tree
## Error: Device /dev/ttyACM0 is already locked
    lsof | grep /dev/ttyACM0
    kill -9 pocess_id