# import launch
import os
import subprocess
import magnet
import get_order
from launch.substitutions import Command
from time import sleep
# from geometry_msgs.msg import PoseStamped
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# import rclpy
# from rclpy.duration import Duration

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

    # rclpy.init()

    # navigator = BasicNavigator()

    # # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # # If autostart, you should `waitUntilNav2Active()` instead.
    # # navigator.lifecycleStartup()

    # # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()

    # # If desired, you can change or load the map as well
    # # navigator.changeMap('/path/to/map.yaml')

    # # You may use the navigator to clear or obtain costmaps
    # # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # # global_costmap = navigator.getGlobalCostmap()
    # # local_costmap = navigator.getLocalCostmap()

    # # Go to our demos first goal pose
    # goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = -2.0
    # goal_pose.pose.position.y = -0.5
    # goal_pose.pose.orientation.w = 1.0

    # # sanity check a valid path exists
    # # path = navigator.getPath(initial_pose, goal_pose)

    # navigator.goToPose(goal_pose)

    # i = 0
    # while not navigator.isTaskComplete():
    #     ################################################
    #     #
    #     # Implement some code here for your application!
    #     #
    #     ################################################

    #     # Do something with the feedback
    #     i = i + 1
    #     feedback = navigator.getFeedback()
    #     if feedback and i % 5 == 0:
    #         print('Estimated time of arrival: ' + '{0:.0f}'.format(
    #               Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
    #               + ' seconds.')

    #         # Some navigation timeout to demo cancellation
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
    #             navigator.cancelTask()

    #         # Some navigation request change to demo preemption
    #         if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    #             goal_pose.pose.position.x = -3.0
    #             navigator.goToPose(goal_pose)

    # # Do something depending on the return code
    # result = navigator.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
    # else:
    #     print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

if __name__ == '__main__':
    main()