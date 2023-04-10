"""
This file allows the autonomous waiter to determine which
orders are ready to be served. This is done by detecting
which colored block is in front of it
"""

# Python code for Multiple Color Detection
import numpy as np
import cv2
import time

"""
f is focal length of the camera
w is width in meters in real life
p is width in pixels in the image
returns distance to object
"""
def distance_to_obj(f, w, p):
    return w*f/p

"""
given x and y pixel coordinates,
determine if the robot needs to
change the yaw to center the obj
in the middle of the pixel
"""
def rotate_to_obj(img, center_obj, dist_obj):
    img_h, img_w = img.shape
    cam_center = (img_h/2, img_w/2)
    return np.arccos((cam_center[0] - center_obj[0])/dist_obj)

def track_order():
    focal_len = 1
    obj_width = 1
    obj_dist = 1
    obj_angle = 1

    # Capturing video through webcam
    webcam = cv2.VideoCapture(0)

    # Capture video through file
    #webcam = cv2.VideoCapture('/home/pi-tcb/auto-waiter/tests/test-yellow.h264')

    # Write video to save results
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('/home/ros2-autonomous-waiter/output.avi', fourcc, 20.0, (640,  480))

    if not webcam.isOpened():
        print("Cannot open camera")
        exit()

    time.sleep(3)

    # Start a while loop
    while(1):

        # Reading the video from the
        # webcam in image frames
        ret, imageFrame = webcam.read()

        if ret == False:
            print('failed to read webcam')
            break

        assert imageFrame is not None

        # # Convert the imageFrame in
        # # BGR(RGB color space) to
        # # HSV(hue-saturation-value)
        # # color space
        # hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

        # # Set range for yellow color and
        # # define mask
        # yellow_lower = np.array([20, 100, 100], np.uint8)
        # yellow_upper = np.array([30, 255, 255], np.uint8)
        # yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)


        # """
        # # Set range for red color and
        # # define mask
        # red_lower = np.array([136, 87, 111], np.uint8)
        # red_upper = np.array([180, 255, 255], np.uint8)
        # red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        # # Set range for green color and
        # # define mask
        # green_lower = np.array([25, 52, 72], np.uint8)
        # green_upper = np.array([102, 255, 255], np.uint8)
        # green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        # # Set range for blue color and
        # # define mask
        # blue_lower = np.array([94, 80, 2], np.uint8)
        # blue_upper = np.array([120, 255, 255], np.uint8)
        # blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
        # """
        # # Morphological Transform, Dilation
        # # for each color and bitwise_and operator
        # # between imageFrame and mask determines
        # # to detect only that particular color
        # kernel = np.ones((5, 5), "uint8")

        # # For yellow color
        # yellow_mask = cv2.dilate(yellow_mask, kernel)
        # res_yellow = cv2.bitwise_and(imageFrame, imageFrame,
        #                         mask = yellow_mask)
        # """
        # # For red color
        # red_mask = cv2.dilate(red_mask, kernel)
        # res_red = cv2.bitwise_and(imageFrame, imageFrame,
        #                         mask = red_mask)

        # # For green color
        # green_mask = cv2.dilate(green_mask, kernel)
        # res_green = cv2.bitwise_and(imageFrame, imageFrame,
        #                             mask = green_mask)

        # # For blue color
        # blue_mask = cv2.dilate(blue_mask, kernel)
        # res_blue = cv2.bitwise_and(imageFrame, imageFrame,
        #                         mask = blue_mask)
        # """
        # # Creating contour to track yellow color
        # contours, hierarchy = cv2.findContours(yellow_mask,
        #                                     cv2.RETR_TREE,
        #                                     cv2.CHAIN_APPROX_SIMPLE)

        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if(area > 300):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         imageFrame = cv2.rectangle(imageFrame, (x, y),
        #                                 (x + w, y + h),
        #                                 (0, 0, 255), 2)

        #         cv2.putText(imageFrame, "Yellow Color", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))
        #         obj_dist = distance_to_obj(focal_len, obj_width, w)
        #         print(f'distance to object = {obj_dist} meters')
        #         obj_angle = rotate_to_obj(pic, (x,y), obj_dist)
        #         print(f'need to rotate by {obj_angle} radians')
        #         # TODO add LCM output


        # """
        # # Creating contour to track red color
        # contours, hierarchy = cv2.findContours(red_mask,
        #                                     cv2.RETR_TREE,
        #                                     cv2.CHAIN_APPROX_SIMPLE)

        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if(area > 300):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         imageFrame = cv2.rectangle(imageFrame, (x, y),
        #                                 (x + w, y + h),
        #                                 (0, 0, 255), 2)

        #         cv2.putText(imageFrame, "Red color", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))

        # # Creating contour to track green color
        # contours, hierarchy = cv2.findContours(green_mask,
        #                                     cv2.RETR_TREE,
        #                                     cv2.CHAIN_APPROX_SIMPLE)

        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if(area > 300):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         imageFrame = cv2.rectangle(imageFrame, (x, y),
        #                                 (x + w, y + h),
        #                                 (0, 255, 0), 2)

        #         cv2.putText(imageFrame, "Green color", (x, y),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0, (0, 255, 0))

        # # Creating contour to track blue color
        # contours, hierarchy = cv2.findContours(blue_mask,
        #                                     cv2.RETR_TREE,
        #                                     cv2.CHAIN_APPROX_SIMPLE)
        # for pic, contour in enumerate(contours):
        #     area = cv2.contourArea(contour)
        #     if(area > 300):
        #         x, y, w, h = cv2.boundingRect(contour)
        #         imageFrame = cv2.rectangle(imageFrame, (x, y),
        #                                 (x + w, y + h),
        #                                 (255, 0, 0), 2)

        #         cv2.putText(imageFrame, "Blue color", (x, y),
        #                     cv2.FONT_HERSHEY_SIMPLEX,
        #                     1.0, (255, 0, 0))
        # """

        # save frame
        #out.write(imageFrame)

        # Program Termination
        cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break

track_order()
