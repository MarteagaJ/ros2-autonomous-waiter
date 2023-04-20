# import launch
import os
import subprocess
# import camera
# import magnet
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
    while(True):
        set_goal_pose(float(0.08), float(0), float(0), float(0))
        sleep(300)
        # for i in range(5):
        #     sleep(2)
        #     set_velocity(float(i), float(i))
            # set_goal_pose(float(i), float(i), float(i), float(i))
        # set_velocity(float(0.3), float(0))

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