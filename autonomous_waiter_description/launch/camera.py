import numpy as np
import cv2
import time
import math
import pdb

"""
focal_length of the camera                      (cm)
Height_real is the height of the object         (cm)
height_projection = pixel_size * height_pixels  (cm)
returns distance to object                      (cm)
using the pinhole project formula
because we have a known height of the object
"""
def distance_to_obj(focal_length, height_real, height_projection):
    return height_real*focal_length/height_projection/fudge_factor

"""
given x and y pixel coordinates,
determine if the robot needs to
change the yaw to center the obj
in the middle of the pixel
"""
def rotate_to_obj(img, center_obj, dist_obj):
    img_h, img_w, img_ch = img.shape
    cam_center = (int(img_w/2), int(img_h/2))
    cam_center_x, cam_center_y = cam_center
    x_obj, y_obj = center_obj   # projected coordiantes

    # draw centers for visual reference
    img = cv2.circle(img, center_obj, 5, (0, 0, 255), 1)
    img = cv2.circle(img, cam_center, 5, (255, 0, 0), 1)

    # determine focal length in pixels
    width_sensor = pixel_size * img_w
    f = int(focal_len * img_w / width_sensor / fudge_factor)   # focal length in pixels

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

pixel_size = 0.00014    # cm/pixel
focal_len = 0.36        # length in cm
obj_height = 2.54       # width in cm (1 inch foam block)
fudge_factor = 3.3      # made up magic number to deal with the compression of video resolution
obj_dist = None
obj_angle = None


# Reading the video from the
# webcam in image frames
# imageFrame = cv2.imread('yellow_20cm.jpg')
# plt.imshow(imageFrame)
# plt.savefig("input-meas.jpg")

# if ret == False:
#     print('failed to read webcam')
#     break

# Reading the video from the
# webcam in image frames
webcam = cv2.VideoCapture(0)
ret, imageFrame = webcam.read()

assert imageFrame is not None
print(np.asarray(imageFrame).shape)
# Convert the imageFrame in
# BGR(RGB color space) to
# HSV(hue-saturation-value)
# color space
#imageFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2RGB)
hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2HSV)

# Set range for yellow color and
# define mask
yellow_lower = np.array([25, 100, 100], np.uint8)
yellow_upper = np.array([35, 255, 255], np.uint8)
yellow_mask = cv2.inRange(hsvFrame, yellow_lower, yellow_upper)
"""
# Set range for red color and
# define mask
red_lower = np.array([136, 87, 111], np.uint8)
red_upper = np.array([180, 255, 255], np.uint8)
red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

# Set range for green color and
# define mask
green_lower = np.array([25, 52, 72], np.uint8)
green_upper = np.array([102, 255, 255], np.uint8)
green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

# Set range for blue color and
# define mask
blue_lower = np.array([94, 80, 2], np.uint8)
blue_upper = np.array([120, 255, 255], np.uint8)
blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
"""
# Morphological Transform, Dilation
# for each color and bitwise_and operator
# between imageFrame and mask determines
# to detect only that particular color
kernel = np.ones((5, 5), "uint8")

# For yellow color
yellow_mask = cv2.dilate(yellow_mask, kernel)
res_yellow = cv2.bitwise_and(imageFrame, imageFrame,
                        mask = yellow_mask)
"""
# For red color
red_mask = cv2.dilate(red_mask, kernel)
res_red = cv2.bitwise_and(imageFrame, imageFrame,
                        mask = red_mask)

# For green color
green_mask = cv2.dilate(green_mask, kernel)
res_green = cv2.bitwise_and(imageFrame, imageFrame,
                            mask = green_mask)

# For blue color
blue_mask = cv2.dilate(blue_mask, kernel)
res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                        mask = blue_mask)
"""

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
        obj_dist = distance_to_obj(focal_len, obj_height, h * pixel_size)
        print(f'distance: \t {obj_dist:.2f} cm')
        cv2.putText(imageFrame, f'distance = {obj_dist:.2f} cm', (5,25), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))

        obj_angle = rotate_to_obj(imageFrame, (int(x+(w/2)),int(y+(h/2))), obj_dist)
        print(f'angle: \t\t {obj_angle:.2f} degrees')
        cv2.putText(imageFrame, f'angle = {obj_angle:.2f} degrees', (5,50), cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 255, 255))
        print()
        # TODO add LCM output


"""
# Creating contour to track red color
contours, hierarchy = cv2.findContours(red_mask,
                                    cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area > 300):
        x, y, w, h = cv2.boundingRect(contour)
        imageFrame = cv2.rectangle(imageFrame, (x, y),
                                (x + w, y + h),
                                (0, 0, 255), 2)

        cv2.putText(imageFrame, "Red color", (x, y),cv2.FONT_HERSHEY_SIMPLEX, 1.0,(0, 0, 255))

# Creating contour to track green color
contours, hierarchy = cv2.findContours(green_mask,
                                    cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_SIMPLE)

for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area > 300):
        x, y, w, h = cv2.boundingRect(contour)
        imageFrame = cv2.rectangle(imageFrame, (x, y),
                                (x + w, y + h),
                                (0, 255, 0), 2)

        cv2.putText(imageFrame, "Green color", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0))

# Creating contour to track blue color
contours, hierarchy = cv2.findContours(blue_mask,
                                    cv2.RETR_TREE,ls
                                    cv2.CHAIN_APPROX_SIMPLE)
for pic, contour in enumerate(contours):
    area = cv2.contourArea(contour)
    if(area > 300):
        x, y, w, h = cv2.boundingRect(contour)
        imageFrame = cv2.rectangle(imageFrame, (x, y),
                                (x + w, y + h),
                                (255, 0, 0), 2)

        cv2.putText(imageFrame, "Blue color", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 0, 0))
"""

cv2.imwrite('output.jpg', imageFrame)
