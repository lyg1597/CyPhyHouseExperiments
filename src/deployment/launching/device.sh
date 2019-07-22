IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')
export ROS_IP=$IP
export ROS_MASTER_URI=http://${IP}:11311

echo $ROS_IP
echo $ROS_MASTER_URI
roscore
