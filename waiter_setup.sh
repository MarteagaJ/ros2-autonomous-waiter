sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
gnome-terminal --command="bash -c 'cd ~/Desktop/teleop-pico/shim_binary/; sudo ./shim; $SHELL'"
gnome-terminal --command="bash -c 'cd ~/Desktop/teleop-pico/timesync_binary/; ./timesync; $SHELL'"
gnome-terminal --command="bash -c 'cd ~/Desktop/ros2-autonomous-waiter/; colcon build --symlink-install; . install/setup.bash; $SHELL'"
gnome-terminal --command="bash -c 'cd ~/Desktop/ros2-autonomous-waiter/; . install/setup.bash; $SHELL'"
gnome-terminal --command="bash -c 'cd ~/Desktop/ros2-autonomous-waiter/autonomous_waiter_description/; $SHELL'"