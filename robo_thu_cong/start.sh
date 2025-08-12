#!/bin/bash

# Giải phóng port 8888 nếu bị chiếm
PID=$(lsof -ti:8888)
if [ -n "$PID" ]; then
    echo "Đang dừng tiến trình chiếm port 8888 (PID: $PID)..."
    kill -9 $PID
fi

# Cửa sổ 1 - micro_ros_agent
gnome-terminal --title="micro_ros_agent" \
    -- bash -c "cd ~/Robocup/Robot_thu_cong && \
                source /opt/ros/humble/setup.bash && source install/setup.bash && \
                export ROS_DOMAIN_ID=0 && \
                ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash"

# Cửa sổ 2 - joystick
gnome-terminal --title="joystick" \
    -- bash -c "cd ~/Robocup/Robot_thu_cong && \
                source /opt/ros/humble/setup.bash && source install/setup.bash && \
                export ROS_DOMAIN_ID=0 && \
                ros2 run control joystick; exec bash"

# Cửa sổ 3 - sim_fth
gnome-terminal --title="sim_fth" \
    -- bash -c "cd ~/Robocup/Robot_thu_cong && \
                source /opt/ros/humble/setup.bash && source install/setup.bash && \
                export ROS_DOMAIN_ID=0 && \
                sleep 3 && \
                ros2 run sim_robot sim_fth; exec bash"

# Cửa sổ 4 - echo topic /esp_vel (đếm ngược 3s)
gnome-terminal --title="echo_esp_vel" \
    -- bash -c "cd ~/Robocup/Robot_thu_cong && \
                source /opt/ros/humble/setup.bash && source install/setup.bash && \
                export ROS_DOMAIN_ID=0 && \
                rqt_graph "
