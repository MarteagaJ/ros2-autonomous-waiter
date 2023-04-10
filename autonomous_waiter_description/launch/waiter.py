import keyboard
import launch
from launch.substitutions import Command
import os

def main(args=None):

    while(True):
        if keyboard.is_pressed("0"):
            Command(['ros2 param set /cmd_vel_unstamped 0'])
        elif keyboard.is_pressed("1"):
            Command(['ros2 param set /cmd_vel_unstamped 1'])
        elif keyboard.is_pressed("2"):
            Command(['ros2 param set /cmd_vel_unstamped 2'])
        elif keyboard.is_pressed("3"):
            Command(['ros2 param set /cmd_vel_unstamped 3'])
        elif keyboard.is_pressed("4"):
            Command(['ros2 param set /cmd_vel_unstamped 4'])
        elif keyboard.is_pressed("5"):
            Command(['ros2 param set /cmd_vel_unstamped 5'])
        elif keyboard.is_pressed("6"):
            Command(['ros2 param set /cmd_vel_unstamped 6'])
        elif keyboard.is_pressed("7"):
            Command(['ros2 param set /cmd_vel_unstamped 7'])
        elif keyboard.is_pressed("8"):
            Command(['ros2 param set /cmd_vel_unstamped 8'])
        elif keyboard.is_pressed("9"):
            Command(['ros2 param set /cmd_vel_unstamped 9'])
        else:
            Command(['ros2 param set /cmd_vel_unstamped 0'])

if __name__ == '__main__':
    main()