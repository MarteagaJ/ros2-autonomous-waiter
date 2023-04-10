import keyboard
import launch
import os
import detect_order
import magnet
from launch.substitutions import Command


def main(args=None):

    while(True):
        magnet.magnet_test()

        if keyboard.is_pressed("0"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 0'])
        elif keyboard.is_pressed("1"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 1'])
        elif keyboard.is_pressed("2"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 2'])
        elif keyboard.is_pressed("3"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 3'])
        elif keyboard.is_pressed("4"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 4'])
        elif keyboard.is_pressed("5"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 5'])
        elif keyboard.is_pressed("6"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 6'])
        elif keyboard.is_pressed("7"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 7'])
        elif keyboard.is_pressed("8"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 8'])
        elif keyboard.is_pressed("9"):
            Command(['ros2 param set /cmd_vel_publisher forward_vel 9'])
        else:
            Command(['ros2 param set /cmd_vel_publisher forward_vel 0'])

if __name__ == '__main__':
    main()