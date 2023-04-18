# import keyboard
# import launch
import os
import subprocess
# import detect_order
# import magnet
from launch.substitutions import Command
from time import sleep
# from pynput import keyboard

# def on_press(key):
#     try:
#         print("press")
        # if key.char == '0':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 0'])
        # elif key.char == '1':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 1'])
        # elif key.char == '2':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 2'])
        # elif key.char == '3':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 3'])
        # elif key.char == '4':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 4'])
        # elif key.char == '5':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 5'])
        # elif key.char == '6':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 6'])
        # elif key.char == '7':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 7'])
        # elif key.char == '8':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 8'])
        # elif key.char == '9':
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 9'])
        # else:
        #     print('alphanumeric key {0} pressed'.format(
        #     key.char))
        #     # output=Command(['ros2 param set /cmd_vel_publisher forward_vel 0'])
    # except AttributeError:
    #     print('special key {0} pressed'.format(
    #         key))

# def on_release(key):
#     print('{0} released'.format(
#         key))
#     if key == keyboard.Key.esc:
#         # Stop listener
#         return False

def set_velocity(x_velocity: float, z_velocity: float):
    cmd_str = "ros2 topic pub --once /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist \"" + "{" + "linear: " + "{" + "x: {}, y: 0.0, z: 0.0".format(x_velocity) + "}" + ", angular: " + "{" + "x: 0.0, y: 0.0, z: {}".format(z_velocity) + "}" + "}\""
    # print(cmd_str)
    # cmd_str = "ros2 param set /cmd_vel_publisher forward_vel {}".format(float(velocity))
    subprocess.run(cmd_str, shell=True)

def main(args=None):
    # listener = keyboard.Listener(
    # on_press=on_press,
    # on_release=on_release)

    # listener.start()

    # Collect events until released
    # with keyboard.Listener(
    #         on_press=on_press,
    #         on_release=on_release) as listener:
    #     listener.join()

    # set_velocity(float(2)/10.0, 0.0)

    while(True):
        # print("test")
        for i in range(9):
            # sleep(4)
            # os.system('cmd "ros2 param set /cmd_vel_publisher forward_vel {}"'.format(i))
            # output=Command(['ros2 param set /cmd_vel_publisher forward_vel {}'.format(float(i))])
            set_velocity(float(i)/10.0, 0.0)
            # print("Current Velocity Set: " + "{" + "linear: " + "{" + "x: {}, y: 0.0, z: 0.0".format(float(i)/10.0) + "}" + ", angular: " + "{" + "x: 0.0, y: 0.0, z: {}".format(0.0) + "}" + "}\"")
            # HELPFUL TOPIC INTROSPECTION COMMAND: ros2 topic echo <topic_name>
        # magnet.magnet_test()

if __name__ == '__main__':
    main()