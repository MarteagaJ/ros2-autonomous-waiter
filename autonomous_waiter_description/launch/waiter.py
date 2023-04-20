# import launch
import os
import subprocess
import camera
import magnet
from launch.substitutions import Command
from time import sleep

def set_velocity(x_velocity: float, z_velocity: float):
    cmd_str = "ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \"" + "{" + "linear: " + "{" + "x: {}, y: 0.0, z: 0.0".format(x_velocity) + "}" + ", angular: " + "{" + "x: 0.0, y: 0.0, z: {}".format(z_velocity) + "}" + "}\""
    # print(cmd_str)
    # cmd_str = "ros2 param set /cmd_vel_publisher forward_vel {}".format(float(velocity))
    subprocess.run(cmd_str, shell=True)

def main(args=None):
    while(True):
        set_velocity(float(0.3), float(0))

if __name__ == '__main__':
    main()