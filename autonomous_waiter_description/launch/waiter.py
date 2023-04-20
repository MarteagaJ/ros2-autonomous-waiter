# import launch
import os
import subprocess
import magnet
import get_order
from launch.substitutions import Command
from time import sleep

def set_velocity(x_velocity: float, z_velocity: float):
    cmd_str = "ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \"" + "{" + "linear: " + "{" + "x: {}, y: 0.0, z: 0.0".format(x_velocity) + "}" + ", angular: " + "{" + "x: 0.0, y: 0.0, z: {}".format(z_velocity) + "}" + "}\""
    # print(cmd_str)
    # cmd_str = "ros2 param set /cmd_vel_publisher forward_vel {}".format(float(velocity))
    subprocess.run(cmd_str, shell=True)

def set_goal_pose(x: float, y: float, z: float, theta: float):
    # cmd_str = "ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}
    header_str = "header: " + "{" + "stamp: " + "{" + "sec: {}, nanosec: {}".format(int(0), int(0)) + "}" + ", frame_id: \"map\"" + "}"
    pose_str = "pose: " + "{" + "position: " + "{" + "x: {}, y: {}, z: {}".format(x, y, z) + "}" + ", orientation: " + "{" + "x: 0.0, y: 0.0, z: {}, w: 1.0".format(theta) + "}" + "}"
    cmd_str = "ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \"" + "{" + "{}, {}".format(header_str, pose_str) + "}" + "\""
    # print(cmd_str)
    # cmd_str = "ros2 param set /cmd_vel_publisher forward_vel {}".format(float(velocity))
    subprocess.run(cmd_str, shell=True)    

def main(args=None):
    pixel_size = 0.00014
    focal_length = 0.36
    pin = 18
    camera = get_order.Get_Order(pix_siz=pixel_size, foc_len=focal_length)
    mag = magnet.Magnet(pin)
    
    # key poses
    pickup_window = ()
    yellow_table = ()
    red_table = ()

    while(True):
        # assuming we will start at the pickup_window

        # detect the order
        color, dist, angle = camera.get_order()
        mag.magnet_on()
        # deliver the order
        if color == 'y':
            print("detected yellow")
            set_goal_pose(yellow_table)
        elif color == 'r':
            print("detected red")
            set_goal_pose(red_table)
        else:
            print("nothing detected")
            mag.magnet_off()
            set_goal_pose(pickup_window)
            continue

        # assuming we got to the table
        mag.magnet_off()

        # return to pickup_window
        set_goal_pose(pickup_window)

if __name__ == '__main__':
    main()