docker exec -it env-zenith-1 bash


ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval "{message_id: 32, message_rate: 20.0}" 


export UID=$(id -u)
export GID=$(id -g)
export XAUTHORITY=${XAUTHORITY:-/home/phil/.Xauthority}
docker compose up -d


source install/setup.bash
ros2 run solution_du_siecle solution 

colcon build --packages-select solution_du_siecle

tcp:192.168.0.35:5762

ros2 run mavros mavros_node --ros-args -p fcu_url:=tcp://192.168.0.35:5762 -p tgt_system:=1 -p tgt_component:=1




3 nodes:
1. Send Basic Commands MAVROS (arm, takeoff, RTL)
2. Mission (send commands in the right order and timers) and also send message to topic /arrival with string "Philippe"
3. Tracking the ballon listening to topic /Ballon_pose

Messages : geometry_msgs/PoseStamped, ENU.