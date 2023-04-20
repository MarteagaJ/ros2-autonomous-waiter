import numpy as np
import cv2
import time
import math
import lcm
from lcmtypes import camera_t

class Get_Order:
    pixel_size = 0.00014     # cm/pixel
    focal_len = 0.36        # length in cm
    obj_height = 2.54       # width in cm (1 inch foam block)
    fudge_factor = 3.3      # made up magic number to deal with the compression of video resolution

    """
    constructor
    """
    def __init__(self, pix_siz=0.00014, foc_len=0.36):
        self.pixel_size = pix_siz
        self.focal_len = foc_len
    """
    focal_length of the camera                      (cm)
    Height_real is the height of the object         (cm)
    height_projection = pixel_size * height_pixels  (cm)
    returns distance to object                      (cm)
    using the pinhole project formula
    because we have a known height of the object
    """
    def distance_to_obj(self, focal_length, height_real, height_projection):
        return height_real*focal_length/height_projection/self.fudge_factor

    """
    given x and y pixel coordinates,
    determine if the robot needs to
    change the yaw to center the obj
    in the middle of the pixel
    """
    def rotate_to_obj(self, img, center_obj, dist_obj):
        img_h, img_w, img_ch = img.shape
        cam_center = (int(img_w/2), int(img_h/2))
        cam_center_x, cam_center_y = cam_center
        x_obj, y_obj = center_obj   # projected coordiantes

        # draw centers for visual reference
        img = cv2.circle(img, center_obj, 5, (0, 0, 255), 1)
        img = cv2.circle(img, cam_center, 5, (255, 0, 0), 1)

        # determine focal length in pixels
        width_sensor = self.pixel_size * img_w
        f = int(self.focal_len * img_w / width_sensor / self.fudge_factor)   # focal length in pixels

        # camera calibration matrix
        K = np.array([[f, 0, cam_center_x],
                    [0, f, cam_center_y],
                    [0, 0, 1]])

        # rays to real world coordinates
        ray_obj = np.linalg.inv(K).dot([x_obj, y_obj, 1.0])
        ray_center = np.linalg.inv(K).dot([cam_center_x, cam_center_y, 1.0])

        radians = math.asin((ray_center[0] - ray_obj[0])/np.linalg.norm(ray_obj))
        degrees = radians * 180/math.pi

        return degrees
    
    def get_order(self):
        done = False
        # Capturing video through webcam
        webcam = cv2.VideoCapture(0)

        if not webcam.isOpened():
            print("Cannot open camera")
            exit()

        lc = lcm.LCM()
        output = camera_t()

        # Start a while loop
        while not done:

            # Reading the video from the
            # webcam in image frames
            ret, imageFrame = webcam.read()

            if ret == False:
                print('failed to read webcam')
                break

            # Convert the imageFrame in
            # BGR(RGB color space) to
            # HSV(hue-saturation-value)
            # color space
            hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

            # Set range for yellow color and
            # define mask
            yellow_lower = np.array([20, 100, 100], np.uint8)
            yellow_upper = np.array([30, 255, 255], np.uint8)
            yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)
        
            # Morphological Transform, Dilation
            # for each color and bitwise_and operator
            # between imageFrame and mask determines
            # to detect only that particular color
            kernel = np.ones((5, 5), "uint8")

            # For yellow color
            yellow_mask = cv2.dilate(yellow_mask, kernel)
            res_yellow = cv2.bitwise_and(imageFrame, imageFrame,
                                    mask = yellow_mask)

            # Creating contour to track yellow color
            contours, hierarchy = cv2.findContours(yellow_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if(area > 300):
                    x, y, w, h = cv2.boundingRect(contour)
                    imageFrame = cv2.rectangle(imageFrame, (x, y),
                                            (x + w, y + h),
                                            (0, 0, 255), 2)

                    cv2.putText(imageFrame, "yellow Color", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))
                    print(f'h = {h}, w = {w}')
                    obj_dist = self.distance_to_obj(self.focal_len, self.obj_height, h * self.pixel_size)
                    print(f'distance: \t {obj_dist:.2f} cm')
                    cv2.putText(imageFrame, f'distance = {obj_dist:.2f} cm', (5,25), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))

                    obj_angle = self.rotate_to_obj(imageFrame, (int(x+(w/2)),int(y+(h/2))), obj_dist)
                    print(f'angle: \t\t {obj_angle:.2f} degrees')
                    cv2.putText(imageFrame, f'angle = {obj_angle:.2f} degrees', (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))
                    print()

                    # lcm output
                    output.utime = int(time.time() * 1e6)
                    output.dist = obj_dist
                    output.theta = obj_angle
                    output.color = 'y'
                    lc.publish("CAMERA", output.encode())

                    if (imageFrame.shape[1] - y) < 5:
                        done = True

                    time.sleep(0.1) # time in seconds

